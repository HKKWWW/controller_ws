import numpy as np
import time
from loguru import logger


# ---------------- 运行时间修饰器 -----------------
def calculate_time(func):
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        logger.info(f"{func.__name__} 运行时间：{time.time() - start:.4f} 秒")
        return result
    return wrapper


# ---------------- 一维定位（两个基站） -----------------
def trilateration_1d(anchors, distances, perturb_eps=0.01):
    a1, a2 = np.array(anchors[0]), np.array(anchors[1])
    d1, d2 = distances

    vec = a2 - a1
    dist = np.linalg.norm(vec)
    direction = vec / dist if dist != 0 else np.array([1.0, 0.0, 0.0])

    try:
        x = (d1**2 - d2**2 + dist**2) / (2 * dist)
        h_sq = d1**2 - x**2
        if h_sq < 0:
            raise ValueError("无交点")
        h = np.sqrt(h_sq)
        p_base = a1 + x * direction

        ortho = np.cross(direction, [1, 0, 0]) if np.allclose(direction[:2], [0, 0]) else np.cross(direction, [0, 0, 1])
        ortho /= np.linalg.norm(ortho)
        p1 = p_base + h * ortho
        p2 = p_base - h * ortho

        e1 = abs(np.linalg.norm(p1 - a1) - d1) + abs(np.linalg.norm(p1 - a2) - d2)
        e2 = abs(np.linalg.norm(p2 - a1) - d1) + abs(np.linalg.norm(p2 - a2) - d2)
        return p1 if e1 < e2 else p2

    except ValueError:
        logger.warning("1D 无法直接求交点，尝试扰动")
        best_p, min_error = None, float('inf')
        for eps in np.linspace(-perturb_eps, perturb_eps, 11):
            try:
                new_d1 = d1 + eps
                x = (new_d1**2 - d2**2 + dist**2) / (2 * dist)
                h_sq = new_d1**2 - x**2
                if h_sq < 0:
                    continue
                h = np.sqrt(h_sq)
                p_base = a1 + x * direction
                ortho = np.cross(direction, [1, 0, 0]) if np.allclose(direction[:2], [0, 0]) else np.cross(direction, [0, 0, 1])
                ortho /= np.linalg.norm(ortho)
                for p in [p_base + h * ortho, p_base - h * ortho]:
                    e = abs(np.linalg.norm(p - a1) - d1) + abs(np.linalg.norm(p - a2) - d2)
                    if e < min_error:
                        min_error, best_p = e, p
            except:
                continue
        if best_p is not None:
            return best_p
        raise RuntimeError("一维定位失败")


# ---------------- 二维定位（3基站） -----------------
def trilateration_2d(anchors, distances, max_iter=100, tol=1e-6):
    anchors = np.array(anchors, dtype=float)
    distances = np.array(distances, dtype=float)
    p = np.array([anchors[0][0] + distances[0], anchors[0][1], 0.0], dtype=float)

    for i in range(max_iter):
        diffs = p - anchors
        dists = np.linalg.norm(diffs, axis=1)
        dists = np.where(dists == 0, 1e-10, dists)
        residuals = dists - distances
        J = diffs / dists[:, np.newaxis]

        JTJ = J.T @ J
        delta = -np.linalg.pinv(JTJ) @ J.T @ residuals
        p += delta

        if np.linalg.norm(delta) < tol:
            break

    logger.debug(f"[2D] 迭代轮次：{i}")
    return p


# ---------------- 二维定位（3基站） -----------------
def trilateration_3d(anchors, distances, max_iter=100, tol=1e-6):
    anchors = np.array(anchors, dtype=float)
    distances = np.array(distances, dtype=float)
    p = anchors[0] + np.array([distances[0], 0.0, 0.0])

    for i in range(max_iter):
        diffs = p - anchors
        dists = np.linalg.norm(diffs, axis=1)
        dists = np.where(dists == 0, 1e-10, dists)
        residuals = dists - distances
        J = diffs / dists[:, np.newaxis]

        JTJ = J.T @ J
        delta = -np.linalg.pinv(JTJ) @ J.T @ residuals
        p += delta

        if np.linalg.norm(delta) < tol:
            break

    logger.debug(f"[3D] 迭代轮次：{i}")
    return p


# ---------------- 自动分发定位器 -----------------
# @calculate_time
def trilateration_dispatch(anchors, distances, max_iter=100, tol=1e-6):
    # 过滤掉无效距离
    valid = [(a, d) for a, d in zip(anchors, distances) if d is not None]
    if len(valid) < 2:
        raise ValueError("至少需要两个有效距离")
    valid_anchors, valid_distances = zip(*valid)

    if len(valid_anchors) == 2:
        # logger.info("使用1D方法进行计算")
        return trilateration_1d(valid_anchors, valid_distances)
    elif len(valid_anchors) == 3:
        # logger.info("使用2D方法进行计算")
        return trilateration_2d(valid_anchors, valid_distances, max_iter, tol)
    else:
        # logger.info("使用3D方法进行计算")
        return trilateration_3d(valid_anchors, valid_distances, max_iter, tol)


# ---------------- 单元测试 -----------------
def test_trilateration_1d():
    anchors = [[0.0, 0.0, 0.0], [2.5, 0.0, 0.0]]
    true_pos = np.array([1.0, 0.0, 0.0])
    distances = [np.linalg.norm(true_pos - np.array(a)) for a in anchors]

    est = trilateration_1d(anchors, distances)
    err = np.linalg.norm(est - true_pos)
    print(f"[1D] 真实: {true_pos}, 估计: {est}, 误差: {err:.6f}")


def test_trilateration_dispatch():
    anchors = [
        [0, 0, 0],
        [2, 0, 0],
        [0, 2, 0],
        [2, 2, 0]
    ]
    true_pos = np.array([1.0, 1.0, 0.0])
    distances = [np.linalg.norm(true_pos - np.array(a)) for a in anchors]

    est = trilateration_dispatch(anchors, distances)
    err = np.linalg.norm(est - true_pos)
    print(f"[Dispatch] 真实: {true_pos}, 估计: {est}, 误差: {err:.6f}")


if __name__ == '__main__':
    test_trilateration_1d()
    test_trilateration_dispatch()
