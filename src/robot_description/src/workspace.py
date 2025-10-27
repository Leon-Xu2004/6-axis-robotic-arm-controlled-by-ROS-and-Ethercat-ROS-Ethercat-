# Python 3.8 OK
# pip install pinocchio numpy
# (optional) pip install scipy trimesh[all] matplotlib

import numpy as np
import pinocchio as pin
from typing import Union
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# --------- 可选依赖（不存在也不影响点云生成） ---------
_HAVE_SCIPY_TRIMESH = True
try:
    import scipy.spatial as sps  # for convex hull
    import trimesh               # for mesh export
except Exception:
    _HAVE_SCIPY_TRIMESH = False

def sample_uniform_in_limits(lower, upper, n):
    lower = np.array(lower); upper = np.array(upper)
    lo = np.where(np.isfinite(lower), lower, -np.pi)
    up = np.where(np.isfinite(upper), upper,  np.pi)
    return lo + (up - lo) * np.random.rand(n, lower.size)

def compute_workspace_points(
    urdf_path: str,
    ee_link: str = "Link6",
    base_link: str = "base_link",
    n_samples: int = 100_000,
    seed: int = 0,
    tool_offset_xyz: Union[np.ndarray, list, tuple] = (0.0, 0.0, 0.0),
    return_orientations: bool = False,
):
    """
    基于 URDF 的随机采样可达集估计（不含碰撞检测）
    返回：points(N,3)；若 return_orientations=True 还会返回 R(N,3,3)
    """
    np.random.seed(seed)

    # 读取模型（固定基座）
    model = pin.buildModelFromUrdf(urdf_path)
    data  = model.createData()

    q_lower = model.lowerPositionLimit.copy()
    q_upper = model.upperPositionLimit.copy()

    movable_idx = np.where(np.isfinite(q_lower) & np.isfinite(q_upper) & (q_upper - q_lower > 1e-6))[0]
    if movable_idx.size == 0:
        raise RuntimeError("未找到可动关节的有效限位。请检查 URDF。")

    # 帧 ID
    fid_ee   = model.getFrameId(ee_link)
    fid_base = model.getFrameId(base_link)
    if fid_ee == len(model.frames):
        raise ValueError(f"找不到末端链接/帧：{ee_link}")
    if fid_base == len(model.frames):
        raise ValueError(f"找不到基座链接/帧：{base_link}")

    # 采样
    Q = np.zeros((n_samples, model.nq))
    Q[:, movable_idx] = sample_uniform_in_limits(q_lower[movable_idx], q_upper[movable_idx], n_samples)

    points = np.empty((n_samples, 3))
    if return_orientations:
        R_all = np.empty((n_samples, 3, 3))

    tool_offset = np.array(tool_offset_xyz, dtype=float).reshape(3)

    # 正解
    for i in range(n_samples):
        q = Q[i]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        oMf_ee   = data.oMf[fid_ee]
        oMf_base = data.oMf[fid_base]

        bMe = oMf_base.inverse() * oMf_ee
        p = bMe.translation.copy() + bMe.rotation @ tool_offset
        points[i] = p
        if return_orientations:
            R_all[i] = bMe.rotation.copy()

    return (points, R_all) if return_orientations else points

# ---------- 可选：体素化 ----------
def voxelize(points: np.ndarray, voxel_size=0.01):
    idx = np.floor(points / voxel_size).astype(int)
    uniq = np.unique(idx, axis=0)
    centers = (uniq.astype(float) + 0.5) * voxel_size
    return centers

# ---------- 可选：凸包网格 ----------
def quick_hull_mesh(points: np.ndarray):
    if not _HAVE_SCIPY_TRIMESH:
        raise ImportError("需要 scipy 和 trimesh：pip install scipy trimesh[all]")
    hull = sps.ConvexHull(points)
    mesh = trimesh.Trimesh(vertices=points[hull.vertices], faces=hull.simplices, process=False)
    return mesh

if __name__ == "__main__":
    # 按你的实际 URDF 路径修改
    URDF = "/home/leon-xu/eRob3_ws/src/robot_description/urdf/erobot3.urdf"

    pts = compute_workspace_points(
        urdf_path=URDF,
        ee_link="Link6",
        base_link="base_link",
        n_samples=50000,
        seed=42
    )
    print("生成点数：", pts.shape)
    np.save("workspace_points.npy", pts)
    print("保存到 workspace_points.npy")

    # 如需导出凸包，请先安装可选依赖并取消注释：
    # mesh = quick_hull_mesh(pts)
    # mesh.export("workspace_hull.stl")
    # print("已导出 workspace_hull.stl")


    # 载入你生成的点云
    pts = np.load("workspace_points.npy")

    print("点数:", pts.shape)
    print("X范围:", pts[:,0].min(), "→", pts[:,0].max())
    print("Y范围:", pts[:,1].min(), "→", pts[:,1].max())
    print("Z范围:", pts[:,2].min(), "→", pts[:,2].max())

    # 3D 散点图
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pts[:,0], pts[:,1], pts[:,2], s=1, c=pts[:,2], cmap='viridis')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("eRob3 机械臂可达工作空间")

    plt.show()