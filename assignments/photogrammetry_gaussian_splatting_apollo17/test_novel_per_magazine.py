from pathlib import Path
import numpy as np
import json
import subprocess
import struct
import collections

base = Path.home() / "ses598-apollo17"
sparse_dir = base / "colmap_n15" / "sparse" / "0"
splat_config = sorted((base / "splats_n15").rglob("config.yml"))[-1]
output_dir = base / "test_novel_renders"
output_dir.mkdir(exist_ok=True)

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
            num_params = {0: 3, 1: 4, 2: 4, 3: 5, 4: 8, 5: 8, 6: 12, 7: 5, 8: 4, 9: 5, 10: 12}[cam_props[1]]
            params = read_next_bytes(fid, 8 * num_params, "d" * num_params)
            cameras[cam_props[0]] = CameraTuple(cam_props[0], cam_props[1], cam_props[2], cam_props[3], params)
    return cameras

def read_images_binary(path):
    images = {}
    with open(path, "rb") as fid:
        num_images = read_next_bytes(fid, 8, "Q")[0]
        for _ in range(num_images):
            image_props = read_next_bytes(fid, 64, "idddddddi")
            qvec = np.array(image_props[1:5])
            tvec = np.array(image_props[5:8])
            name = ""
            char = read_next_bytes(fid, 1, "c")[0]
            while char != b"\x00":
                name += char.decode("utf-8")
                char = read_next_bytes(fid, 1, "c")[0]
            num_points2d = read_next_bytes(fid, 8, "Q")[0]
            read_next_bytes(fid, 24 * num_points2d, "ddq" * num_points2d)
            images[image_props[0]] = ImageTuple(image_props[0], qvec, tvec, image_props[8], name)
    return images

def qvec_to_rotmat(qvec):
    w, x, y, z = qvec
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])

def look_at_to_c2w(eye, target, up=np.array([0, 1, 0])):
    forward = target - eye
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, up)
    right = right / np.linalg.norm(right)
    new_up = np.cross(right, forward)
    c2w = np.eye(4)
    c2w[:3, 0] = right
    c2w[:3, 1] = -new_up
    c2w[:3, 2] = forward
    c2w[:3, 3] = eye
    return c2w

cameras = read_cameras_binary(sparse_dir / "cameras.bin")
images = read_images_binary(sparse_dir / "images.bin")

mag137_centers = []
mag137_forwards = []
mag138_centers = []
mag138_forwards = []
for img in images.values():
    R = qvec_to_rotmat(img.qvec)
    center = -R.T @ img.tvec
    forward = R.T @ np.array([0, 0, 1])
    if "mag137" in img.name:
        mag137_centers.append(center)
        mag137_forwards.append(forward)
    else:
        mag138_centers.append(center)
        mag138_forwards.append(forward)

mag137_centers = np.array(mag137_centers)
mag138_centers = np.array(mag138_centers)
mag137_forwards = np.array(mag137_forwards)
mag138_forwards = np.array(mag138_forwards)

mag137_centroid = mag137_centers.mean(axis=0)
mag138_centroid = mag138_centers.mean(axis=0)
mag137_radius = np.median(np.linalg.norm(mag137_centers - mag137_centroid, axis=1))
mag138_radius = np.median(np.linalg.norm(mag138_centers - mag138_centroid, axis=1))
mag137_mean_fwd = mag137_forwards.mean(axis=0)
mag137_mean_fwd = mag137_mean_fwd / np.linalg.norm(mag137_mean_fwd)
mag138_mean_fwd = mag138_forwards.mean(axis=0)
mag138_mean_fwd = mag138_mean_fwd / np.linalg.norm(mag138_mean_fwd)

mag137_target = mag137_centroid + mag137_mean_fwd * 3.0
mag138_target = mag138_centroid + mag138_mean_fwd * 3.0

print(f"mag137 centroid={mag137_centroid}, radius={mag137_radius:.3f}, target={mag137_target}")
print(f"mag138 centroid={mag138_centroid}, radius={mag138_radius:.3f}, target={mag138_target}")

test_poses = []
test_poses.append(("mag137_test_a", mag137_centroid, mag137_target))
test_poses.append(("mag137_test_b", mag137_centroid + np.array([0, 0, 0.5]), mag137_target))
test_poses.append(("mag138_test_a", mag138_centroid, mag138_target))
test_poses.append(("mag138_test_b", mag138_centroid + np.array([0, 0, 0.8]), mag138_target))

camera_path = {
    "render_height": 1024,
    "render_width": 1024,
    "camera_path": [],
    "fps": 1,
    "seconds": len(test_poses),
    "smoothness_value": 0,
    "is_cycle": False,
    "crop": None
}

for name, eye, target in test_poses:
    c2w = look_at_to_c2w(eye, target)
    camera_path["camera_path"].append({
        "camera_to_world": c2w.flatten().tolist(),
        "fov": 50,
        "aspect": 1.0
    })

cam_path_file = output_dir / "test_path.json"
with open(cam_path_file, "w") as f:
    json.dump(camera_path, f, indent=2)

print(f"\nWrote camera path to {cam_path_file}")
print(f"Splat config: {splat_config}")
print(f"\nNow run:\nns-render camera-path --load-config {splat_config} --camera-path-filename {cam_path_file} --output-path {output_dir / 'test.mp4'}")
