import numpy as np
import math
import time


"""
    三边定位算法
"""
def trilateration_2d(anchors, distances, max_iter=100, tol=1e-6):
    anchors = np.array(anchors, dtype=float)
    distances = np.array(distances, dtype=float)

    # 检查输入参数
    if len(anchors) != len(distances):
        raise ValueError("锚点数量与距离数量不匹配")
    
    if len(anchors) < 2:
        raise ValueError("至少需要2个锚点")
    
    # 初始化位置估计 - 使用第一个锚点和距离作为初始点
    p = np.array([anchors[0][0] + distances[0], anchors[0][1]], dtype=float)
    
    i = 0
    for i in range(max_iter):
        diffs = p - anchors  # [xi - x0, yi - y0]
        dists = np.linalg.norm(diffs, axis=1)  # 模长

        dists = np.where(dists == 0, 1e-10, dists)  # 将距离为0的情况替换为一个很小的数
        
        residuals = dists - distances  # 求残差
        
        J = diffs / dists[:, np.newaxis]  # 求雅可比矩阵

        # 高斯-牛顿更新 Δp = -(JᵗJ)⁻¹ Jᵗr
        try:
            JTJ = J.T @ J
            # 检查矩阵是否接近奇异
            if np.linalg.cond(JTJ) < 1e12:  # 条件数不太大时使用精确解
                delta = -np.linalg.inv(JTJ) @ J.T @ residuals
            else:  # 否则使用伪逆
                delta = -np.linalg.pinv(JTJ) @ J.T @ residuals
        except np.linalg.LinAlgError:
            # 如果矩阵求逆失败，使用梯度下降步长
            delta = -J.T @ residuals * 0.1  # 使用较小的学习率
        
        p = p + delta  # 优化 p 点

        # 检查收敛性
        if np.linalg.norm(delta) < tol:
            break
            
    print(f"迭代轮次：{i}")
    return p


if __name__ == '__main__':
    anchors = [(0, 0), (2, 0), (0, 2)]     # 三个已知锚点位置（单位：米）
    distances = [1.4, 1.6, 1.5]            # 到每个锚点的距离（含误差）
    try:
        now = time.time()
        result = trilateration_2d(anchors, distances)
        print(f"花费时间为：{time.time() - now}")
        print(f"Estimated position: x = {result[0]:.3f}, y = {result[1]:.3f}")
    except Exception as e:
        print(f"定位计算出错: {e}")
    # p = np.asarray([0.951, 1.022])
    # anchors = np.asarray([[0, 0], [2, 0], [0, 2]])

    # distances = np.linalg.norm(anchors - p, axis=1)

    # print("distances: ", distances)