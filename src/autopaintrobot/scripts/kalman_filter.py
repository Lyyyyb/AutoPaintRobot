import numpy as np
#卡尔曼滤波类
class KalmanFilter:
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x=None):
        # 初始化卡尔曼滤波器的各个参数
        self.n = F.shape[1]  # 状态向量的维度
        self.F = F if F is not None else np.eye(self.n)  # 状态转移矩阵
        self.H = H if H is not None else np.eye(self.n)  # 观测矩阵
        self.B = B if B is not None else np.zeros((self.n, 1))  # 控制输入矩阵
        self.Q = Q if Q is not None else np.eye(self.n)  # 过程噪声协方差矩阵
        self.R = R if R is not None else np.eye(self.n)  # 观测噪声协方差矩阵
        self.P = P if P is not None else np.eye(self.n)  # 估计误差协方差矩阵
        self.x = x if x is not None else np.zeros((self.n, 1))  # 初始状态估计


    def predict(self, u=0):
        # 状态预测
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)  # 根据模型和控制输入更新状态估计
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q  # 更新估计误差协方差
        return self.x  # 返回更新后的状态估计


    def update(self, z):
        # 状态更新
        y = z - np.dot(self.H, self.x)  # 计算测量残差
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))  # 计算残差协方差
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))  # 计算卡尔曼增益
        self.x = self.x + np.dot(K, y)  # 更新状态估计
        I = np.eye(self.n)  # 单位矩阵
        self.P = np.dot(I - np.dot(K, self.H), self.P)  # 更新估计误差协方差
