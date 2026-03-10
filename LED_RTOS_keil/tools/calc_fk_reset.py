#!/usr/bin/env python3
"""
计算dummy-auk机械臂初始姿态的FK矩阵
用于更新motion_controller.c中的T_0_6_reset
"""

import numpy as np

# DH参数（来自dummy-auk）
# DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f)
L_BASE = 0.109      # 109mm
D_BASE = 0.035      # 35mm
L_ARM = 0.146       # 146mm
L_FOREARM = 0.115   # 115mm
D_ELBOW = 0.052     # 52mm
L_WRIST = 0.072     # 72mm

# dummy-auk的初始姿态（度）
REST_POSE = [0, -73, 180, 0, 0, 0]

# DH参数表（根据kinematics.c的定义）
# [a, alpha, d, theta_offset]
DH_PARAMS = [
    [D_BASE,    0,          L_BASE,     np.pi/2],   # J1
    [0,         np.pi/2,    0,          np.pi/2],   # J2
    [L_ARM,     np.pi,      0,          -np.pi/2],  # J3
    [D_ELBOW,   -np.pi/2,   -L_FOREARM, 0],         # J4
    [0,         np.pi/2,    0,          np.pi/2],   # J5
    [0,         np.pi/2,    L_WRIST,    0]          # J6
]

def dh_transform(a, alpha, d, theta):
    """DH变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct,    -st*ca,  st*sa,   a*ct],
        [st,     ct*ca, -ct*sa,   a*st],
        [0,      sa,     ca,      d],
        [0,      0,      0,       1]
    ])

def forward_kinematics(joint_angles_deg):
    """
    正运动学求解
    joint_angles_deg: 6个关节角度（度）
    返回: 4x4齐次变换矩阵
    """
    # 转换为弧度
    joint_angles_rad = [np.deg2rad(angle) for angle in joint_angles_deg]
    
    # 初始化为单位矩阵
    T = np.eye(4)
    
    # 逐个关节相乘
    for i in range(6):
        a, alpha, d, theta_offset = DH_PARAMS[i]
        theta = joint_angles_rad[i] + theta_offset
        T_i = dh_transform(a, alpha, d, theta)
        T = T @ T_i
    
    return T

def format_matrix_c(T, name="T_0_6_reset"):
    """格式化为C代码"""
    lines = [f"static const float {name}[4][4] = {{"]
    for i in range(4):
        row = "    {" + ", ".join([f"{T[i,j]:8.4f}f" for j in range(4)]) + "}"
        if i < 3:
            row += ","
        lines.append(row)
    lines.append("};")
    return "\n".join(lines)

if __name__ == "__main__":
    print("=" * 60)
    print("Dummy-auk机械臂初始姿态FK计算")
    print("=" * 60)
    
    print(f"\n初始姿态（度）: {REST_POSE}")
    print(f"DH参数: L_BASE={L_BASE*1000}mm, D_BASE={D_BASE*1000}mm, L_ARM={L_ARM*1000}mm")
    print(f"        L_FOREARM={L_FOREARM*1000}mm, D_ELBOW={D_ELBOW*1000}mm, L_WRIST={L_WRIST*1000}mm")
    
    # 计算FK
    T_reset = forward_kinematics(REST_POSE)
    
    print("\n末端位置（米）:")
    print(f"  X = {T_reset[0,3]:.4f} m = {T_reset[0,3]*1000:.2f} mm")
    print(f"  Y = {T_reset[1,3]:.4f} m = {T_reset[1,3]*1000:.2f} mm")
    print(f"  Z = {T_reset[2,3]:.4f} m = {T_reset[2,3]*1000:.2f} mm")
    
    print("\n旋转矩阵:")
    print(T_reset[:3,:3])
    
    print("\n完整变换矩阵:")
    print(T_reset)
    
    print("\n" + "=" * 60)
    print("C代码（复制到motion_controller.c）:")
    print("=" * 60)
    print(format_matrix_c(T_reset))
    
    print("\n" + "=" * 60)
    print("关节偏移量计算:")
    print("=" * 60)
    print("软件零点 = 电机零点 + offset")
    print("要使软件零点对应REST_POSE，offset应该为:")
    print(f"g_joint_offset[6] = {{{', '.join([f'{angle:.1f}f' for angle in REST_POSE])}}};")
