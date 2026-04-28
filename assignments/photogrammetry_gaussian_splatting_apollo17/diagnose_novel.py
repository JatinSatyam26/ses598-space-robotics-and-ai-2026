from pathlib import Path
import numpy as np
import json
import struct
import collections
import subprocess

base = Path.home() / "ses598-apollo17"
sparse_dir = base / "colmap_n15" / "sparse" / "0"

CameraTuple = collections.namedtuple("CameraTuple", ["id", "model", "width", "height", "params"])
ImageTuple = collections.namedtuple("ImageTuple", ["id", "qvec", "tvec", "camera_id", "name"])

def read_next_bytes(fid, num_bytes, format_char_sequence, endian="<"):
    data = fid.read(num_bytes)
    return struct.unpack(endian + format_char_sequence, data)

def read_cameras_binary(path):
    cameras = {}
    with open(path, "rb") as fid:
        num_cameras = read_next_bytes(fid, 8, "Q")[0]
        for _ in range(num_cameras):
            cam_props = read_next_bytes(fid, 24, "iiQQ")
            camera_id = cam_props[0]
            model_id = cam_props[1]
            width = cam_props[2]
            height = cam_props[3]
            num_params = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8, 5: 8, 6: 12, 7: 5, 8: 4, 9: 5, 10: 12}[model_id]
            params = read_next_bytes(fid, 8 * num_params, "d" * num_params)
            cameras[camera_id] = CameraTuple(camera_id, model_id, width, height, params)
    return cameras

def read_images_binary(path):
    images = {}
    with open(path, "rb") as fid:
        num_images = read_next_bytes(fid, 8, "Q")[0]
        for _ in range(num_images):
            image_props = read_next_bytes(fid, 64, "idddddddi")
            image_id = image_props[0]
            qvec = np.array(image_props[1:5])
            tvec = np.array(image_props[5:8])
            camera_id = image_props[8]
            name = ""
            char = read_next_bytes(fid, 1, "c")[0]
            while char != b"\x00":
                name += char.decode("utf-8")
                char = read_next_bytes(fid, 1, "c")[0]
            num_points2d = read_next_bytes(fid, 8, "Q")[0]
            read_next_bytes(fid, 24 * num_points2d, "ddq" * num_points2d)
            images[image_id] = ImageTuple(image_id, qvec, tvec, camera_id, name)
    return images

def qvec_to_rotmat(qvec):
    w, x, y, z = qvec
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])

cameras = read_cameras_binary(sparse_dir / "cameras.bin")
images = read_images_binary(sparse_dir / "images.bin")

print(f"Cameras: {len(cameras)}")
print(f"Images: {len(images)}")
print()

centers = []
forwards = []
for img_id, img in sorted(images.items(), key=lambda kv: kv[1].name):
    R = qvec_to_rotmat(img.qvec)
    t = img.tvec
    center = -R.T @ t
    forward = R.T @ np.array([0, 0, 1])
    centers.append(center)
    forwards.append(forward)
    print(f"{img.name:40s}  center=({center[0]:7.3f},{center[1]:7.3f},{center[2]:7.3f})  cam_id={img.camera_id}")

centers = np.array(centers)
forwards = np.array(forwards)

print()
print("=== Pose cloud statistics ===")
print(f"Centroid:        {centers.mean(axis=0)}")
print(f"Bounding box:    min={centers.min(axis=0)}, max={centers.max(axis=0)}")
print(f"Spread (std):    {centers.std(axis=0)}")
print(f"Mean pairwise distance: {np.mean([np.linalg.norm(centers[i]-centers[j]) for i in range(len(centers)) for j in range(i+1,len(centers))]):.3f}")

mag137_mask = ["mag137" in img.name for img in sorted(images.values(), key=lambda i: i.name)]
mag138_mask = ["mag138" in img.name for img in sorted(images.values(), key=lambda i: i.name)]

mag137_centers = centers[mag137_mask]
mag138_centers = centers[mag138_mask]

print()
print(f"mag137 centroid: {mag137_centers.mean(axis=0)}")
print(f"mag138 centroid: {mag138_centers.mean(axis=0)}")
print(f"Inter-cluster distance: {np.linalg.norm(mag137_centers.mean(axis=0) - mag138_centers.mean(axis=0)):.3f}")
print(f"mag137 intra-cluster mean dist: {np.mean([np.linalg.norm(mag137_centers[i]-mag137_centers[j]) for i in range(len(mag137_centers)) for j in range(i+1,len(mag137_centers))]):.3f}")
print(f"mag138 intra-cluster mean dist: {np.mean([np.linalg.norm(mag138_centers[i]-mag138_centers[j]) for i in range(len(mag138_centers)) for j in range(i+1,len(mag138_centers))]):.3f}")

mean_forward = forwards.mean(axis=0)
mean_forward = mean_forward / np.linalg.norm(mean_forward)
look_at_target = centers.mean(axis=0) + mean_forward * np.linalg.norm(centers.std(axis=0)) * 2
print(f"\nEstimated scene look-at target: {look_at_target}")
