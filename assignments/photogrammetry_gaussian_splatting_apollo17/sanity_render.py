import numpy as np
import open3d as o3d
import struct
import os
os.environ['PYOPENGL_PLATFORM'] = 'egl'
import pyrender
import trimesh
from pathlib import Path
from PIL import Image

def read_images_bin(path):
    images = {}
    with open(path, 'rb') as f:
        n = struct.unpack('<Q', f.read(8))[0]
        for _ in range(n):
            iid = struct.unpack('<I', f.read(4))[0]
            qvec = np.array(struct.unpack('<4d', f.read(32)))
            tvec = np.array(struct.unpack('<3d', f.read(24)))
            cid = struct.unpack('<I', f.read(4))[0]
            name = b''
            while True:
                c = f.read(1)
                if c == b'\x00': break
                name += c
            n_pts = struct.unpack('<Q', f.read(8))[0]
            f.read(n_pts * 24)
            qw, qx, qy, qz = qvec
            R = np.array([
                [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)]
            ])
            images[iid] = dict(name=name.decode(), cam_id=cid, R=R, tvec=tvec)
    return images

def read_cameras_bin(path):
    cameras = {}
    nparams = {0:3,1:4,2:4,3:5,4:8}
    with open(path, 'rb') as f:
        n = struct.unpack('<Q', f.read(8))[0]
        for _ in range(n):
            cid = struct.unpack('<I', f.read(4))[0]
            mid = struct.unpack('<I', f.read(4))[0]
            w = struct.unpack('<Q', f.read(8))[0]
            h = struct.unpack('<Q', f.read(8))[0]
            np_ = nparams[mid]
            params = list(struct.unpack(f'<{np_}d', f.read(8*np_)))
            cameras[cid] = dict(model=mid, W=w, H=h, params=params)
    return cameras

def colmap_pose_to_c2w(R, tvec):
    c2w = np.eye(4)
    c2w[:3, :3] = R.T
    c2w[:3, 3] = -R.T @ tvec
    return c2w

def render_vertex_colored_ply(ply_path, sparse_dir, img_name, out_path, W=800, H=800):
    sparse_dir = Path(sparse_dir)
    cams = read_cameras_bin(sparse_dir / 'cameras.bin')
    imgs = read_images_bin(sparse_dir / 'images.bin')

    chosen = next(img for img in imgs.values() if img['name'] == img_name)
    cam_data = cams[chosen['cam_id']]
    p = cam_data['params']
    m = cam_data['model']
    if m == 1:
        fx, fy, cx, cy = p[0], p[1], p[2], p[3]
    elif m == 2:
        fx = fy = p[0]; cx = p[1]; cy = p[2]
    else:
        fx = fy = p[0]; cx = p[1]; cy = p[2]

    sx = W / cam_data['W']; sy = H / cam_data['H']
    fx_r = fx * sx; fy_r = fy * sy; cx_r = cx * sx; cy_r = cy * sy

    c2w = colmap_pose_to_c2w(chosen['R'], chosen['tvec'])
    opengl_flip = np.diag([1, -1, -1, 1])
    camera_pose = c2w @ opengl_flip

    mesh_o3d = o3d.io.read_triangle_mesh(str(ply_path))
    mesh_o3d.compute_vertex_normals()
    verts = np.asarray(mesh_o3d.vertices)
    tris = np.asarray(mesh_o3d.triangles)
    colors = np.asarray(mesh_o3d.vertex_colors)

    tm = trimesh.Trimesh(vertices=verts, faces=tris, vertex_colors=(colors * 255).astype(np.uint8))
    mesh_pr = pyrender.Mesh.from_trimesh(tm, smooth=False)

    scene = pyrender.Scene(bg_color=[0.2, 0.2, 0.2, 1.0], ambient_light=[0.8, 0.8, 0.8])
    scene.add(mesh_pr)

    camera = pyrender.IntrinsicsCamera(fx=fx_r, fy=fy_r, cx=cx_r, cy=cy_r, znear=0.01, zfar=1000.0)
    scene.add(camera, pose=camera_pose)

    light = pyrender.DirectionalLight(color=[1.0, 1.0, 1.0], intensity=2.0)
    scene.add(light, pose=camera_pose)

    renderer = pyrender.OffscreenRenderer(W, H)
    color, depth = renderer.render(scene, flags=pyrender.RenderFlags.FLAT)
    renderer.delete()

    Image.fromarray(color).save(str(out_path))
    print(f'Sanity render saved: {out_path}')
    img_arr = np.array(Image.fromarray(color))
    print(f'  Pixel stats: mean={img_arr.mean():.1f}  std={img_arr.std():.1f}')
    return img_arr

render_vertex_colored_ply(
    'colmap_n15/dense/textured/textured.ply',
    'colmap_n15/sparse/0',
    'mag137/AS17-137-20907HR.png',
    'colmap_n15/dense/textured/sanity_check.png'
)
