"""
dummy-auk IK/FK 验证脚本 (v2 - 连杆向量法FK)
验证内容:
1. FK正运动学 — REST_POSE → 末端位置 (连杆向量法, 与dummy-auk完全一致)
2. T_0_6_reset矩阵正确性
3. IK逆运动学 — 8组解 + 限位 + 最优选择
4. FK(IK(target)) ≈ target 闭环验证
5. 偏移方向: T_0_6_reset * T_offset vs T_offset * T_0_6_reset
"""
import numpy as np
from math import pi, sin, cos, atan2, acos, sqrt, fabs

# ========== 连杆参数 (米) ==========
L_BASE   = 0.109
D_BASE   = 0.035
L_ARM    = 0.146
L_FOREARM= 0.115
D_ELBOW  = 0.052
L_WRIST  = 0.072

# DH参数: [home_offset, d, a, alpha]
DH = np.array([
    [0.0,       L_BASE,    D_BASE,   -pi/2],
    [-pi/2,     0.0,       L_ARM,     0.0],
    [pi/2,      D_ELBOW,   0.0,       pi/2],
    [0.0,       L_FOREARM, 0.0,      -pi/2],
    [0.0,       0.0,       0.0,       pi/2],
    [0.0,       L_WRIST,   0.0,       0.0],
])

REST_POSE = np.array([0, -73, 180, 0, 0, 0], dtype=float)

# 连杆向量 (与C代码一致)
L1_base  = np.array([D_BASE, -L_BASE, 0.0])
L2_arm   = np.array([L_ARM, 0.0, 0.0])
L3_elbow = np.array([-D_ELBOW, 0.0, L_FOREARM])
L6_wrist = np.array([0.0, 0.0, L_WRIST])

# 关节限位 (度)
JOINT_LIMITS = [
    (-170, 170),
    (-73,  90),
    (35,   180),
    (-180, 180),
    (-120, 120),
    (-720, 720),
]

JOINT_WEIGHTS = [5, 3, 3, 1, 1, 1]
KIN_LIMIT_TOLERANCE = 0.5
RAD_TO_DEG = 57.295777754771045

# IK预计算
l_se_2 = L_ARM * L_ARM
l_se   = L_ARM
l_ew_2 = L_FOREARM**2 + D_ELBOW**2
l_ew   = sqrt(l_ew_2)
atan_e = atan2(D_ELBOW, L_FOREARM)

# ========== FK 正运动学 (连杆向量法, 完全复刻dummy-auk) ==========

def dh_rotation_3x3(q_rad, alpha):
    """DH旋转矩阵 (仅3x3, 与dummy-auk SolveFK一致)"""
    cosq = cos(q_rad)
    sinq = sin(q_rad)
    cosa = cos(alpha)
    sina = sin(alpha)
    return np.array([
        [cosq,  -cosa*sinq,  sina*sinq],
        [sinq,   cosa*cosq, -sina*cosq],
        [0.0,    sina,        cosa     ],
    ])

def fk(joints_deg):
    """正运动学: 6个关节角(度) → 4x4齐次变换矩阵
       使用连杆向量法 (与dummy-auk SolveFK完全一致)"""
    q_rad = np.deg2rad(joints_deg)

    # 每个关节的旋转矩阵
    R = []
    for i in range(6):
        q = q_rad[i] + DH[i, 0]  # 加home_offset
        Ri = dh_rotation_3x3(q, DH[i, 3])
        R.append(Ri)

    # 累积旋转
    R01 = R[0]
    R02 = R01 @ R[1]
    R03 = R02 @ R[2]
    R04 = R03 @ R[3]
    R05 = R04 @ R[4]
    R06 = R05 @ R[5]

    # 位置 = 连杆向量之和 (各自在世界坐标系下)
    L0_bs = R01 @ L1_base
    L0_se = R02 @ L2_arm
    L0_ew = R03 @ L3_elbow
    L0_wt = R06 @ L6_wrist

    P06 = L0_bs + L0_se + L0_ew + L0_wt

    # 组装4x4
    T = np.eye(4)
    T[:3, :3] = R06
    T[:3, 3] = P06
    return T

def fk_position_mm(joints_deg):
    T = fk(joints_deg)
    return T[:3, 3] * 1000

# ========== IK 逆运动学 (几何法, 完全复刻C代码) ==========

def safe_acos(x):
    return acos(max(-1.0, min(1.0, x)))

def solve_ik(T_target_mm, current_angles_deg):
    """IK求解: T_target(位置单位mm) → 8组解(度)"""
    P06 = T_target_mm[:3, 3] / 1000.0
    R06 = T_target_mm[:3, :3].copy()

    L0_wt = R06 @ L6_wrist
    P0_w = P06 - L0_wt

    solutions = np.zeros((8, 6))
    invalid_mask = 0
    solFlag = np.zeros((8, 3), dtype=int)

    pw_xy = sqrt(P0_w[0]**2 + P0_w[1]**2)
    if pw_xy <= 1e-6:
        last_q0 = np.deg2rad(current_angles_deg[0])
        qs = [last_q0, last_q0]
        for i in range(4):
            solFlag[i][0] = -1
            solFlag[4+i][0] = -1
    else:
        qs = [atan2(P0_w[1], P0_w[0]), atan2(-P0_w[1], -P0_w[0])]
        for i in range(4):
            solFlag[i][0] = 1
            solFlag[4+i][0] = 1

    for ind_arm in range(2):
        cosqs = cos(qs[ind_arm] + DH[0, 0])
        sinqs = sin(qs[ind_arm] + DH[0, 0])

        R10 = np.array([
            [ cosqs, sinqs, 0],
            [ 0,     0,    -1],
            [-sinqs, cosqs, 0],
        ])

        P1_w = R10 @ P0_w
        L1_sw = P1_w - L1_base

        l_sw_2 = L1_sw[0]**2 + L1_sw[1]**2
        l_sw = sqrt(l_sw_2)

        qa = np.zeros((2, 2))
        for i in range(2):
            qa[i, 0] = np.deg2rad(current_angles_deg[1])
            qa[i, 1] = np.deg2rad(current_angles_deg[2])

        if fabs(l_se + l_ew - l_sw) <= 1e-6:
            qa[0, 0] = atan2(L1_sw[1], L1_sw[0])
            qa[1, 0] = qa[0, 0]
            qa[0, 1] = 0.0; qa[1, 1] = 0.0
            flag = 0 if l_sw > l_se + l_ew else 1
            for i in range(2):
                solFlag[4*ind_arm+i][1] = flag
                solFlag[4*ind_arm+2+i][1] = flag
        elif fabs(l_sw - fabs(l_se - l_ew)) <= 1e-6:
            qa[0, 0] = atan2(L1_sw[1], L1_sw[0])
            qa[1, 0] = qa[0, 0]
            if ind_arm == 0:
                qa[0, 1] = pi; qa[1, 1] = -pi
            else:
                qa[0, 1] = -pi; qa[1, 1] = pi
            flag = 0 if l_sw < fabs(l_se - l_ew) else 1
            for i in range(2):
                solFlag[4*ind_arm+i][1] = flag
                solFlag[4*ind_arm+2+i][1] = flag
        else:
            atan_a = atan2(L1_sw[1], L1_sw[0])
            cos_acos_a = 0.5 * (l_se_2 + l_sw_2 - l_ew_2) / (l_se * l_sw)
            acos_a = safe_acos(cos_acos_a)
            cos_acos_e = 0.5 * (l_se_2 + l_ew_2 - l_sw_2) / (l_se * l_ew)
            acos_e = safe_acos(cos_acos_e)

            if ind_arm == 0:
                qa[0, 0] = atan_a - acos_a + pi/2
                qa[0, 1] = atan_e - acos_e + pi
                qa[1, 0] = atan_a + acos_a + pi/2
                qa[1, 1] = atan_e + acos_e - pi
            else:
                qa[0, 0] = atan_a + acos_a + pi/2
                qa[0, 1] = atan_e + acos_e - pi
                qa[1, 0] = atan_a - acos_a + pi/2
                qa[1, 1] = atan_e - acos_e + pi
            for i in range(2):
                solFlag[4*ind_arm+i][1] = 1
                solFlag[4*ind_arm+2+i][1] = 1

        for ind_elbow in range(2):
            cosqa0 = cos(qa[ind_elbow, 0] + DH[1, 0])
            sinqa0 = sin(qa[ind_elbow, 0] + DH[1, 0])
            cosqa1 = cos(qa[ind_elbow, 1] + DH[2, 0])
            sinqa1 = sin(qa[ind_elbow, 1] + DH[2, 0])

            R31 = np.array([
                [cosqa0*cosqa1 - sinqa0*sinqa1, cosqa0*sinqa1 + sinqa0*cosqa1, 0],
                [0, 0, 1],
                [cosqa0*sinqa1 + sinqa0*cosqa1, -cosqa0*cosqa1 + sinqa0*sinqa1, 0],
            ])

            R30 = R31 @ R10
            R36 = R30 @ R06

            qw = np.zeros((2, 3))
            for i in range(2):
                qw[i, 0] = np.deg2rad(current_angles_deg[3])
                qw[i, 1] = np.deg2rad(current_angles_deg[4])
                qw[i, 2] = np.deg2rad(current_angles_deg[5])

            WRIST_THRESH = 1e-3
            if R36[2, 2] >= 1.0 - WRIST_THRESH:
                cosqw = 1.0
                qw[0, 1] = 0.0; qw[1, 1] = 0.0
                last_q3 = np.deg2rad(current_angles_deg[3])
                last_q5 = np.deg2rad(current_angles_deg[5])
                if ind_arm == 0:
                    qw[0, 0] = last_q3
                    c = cos(last_q3 + DH[3, 0]); s = sin(last_q3 + DH[3, 0])
                    qw[0, 2] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                    qw[1, 2] = last_q5
                    c = cos(last_q5 + DH[5, 0]); s = sin(last_q5 + DH[5, 0])
                    qw[1, 0] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                else:
                    qw[0, 2] = last_q5
                    c = cos(last_q5 + DH[5, 0]); s = sin(last_q5 + DH[5, 0])
                    qw[0, 0] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                    qw[1, 0] = last_q3
                    c = cos(last_q3 + DH[3, 0]); s = sin(last_q3 + DH[3, 0])
                    qw[1, 2] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                solFlag[4*ind_arm + 2*ind_elbow + 0][2] = -1
                solFlag[4*ind_arm + 2*ind_elbow + 1][2] = -1
            elif R36[2, 2] <= -1.0 + WRIST_THRESH:
                cosqw = -1.0
                if ind_arm == 0:
                    qw[0, 1] = pi; qw[1, 1] = -pi
                else:
                    qw[0, 1] = -pi; qw[1, 1] = pi
                last_q3 = np.deg2rad(current_angles_deg[3])
                last_q5 = np.deg2rad(current_angles_deg[5])
                if ind_arm == 0:
                    qw[0, 0] = last_q3
                    c = cos(last_q3 + DH[3, 0]); s = sin(last_q3 + DH[3, 0])
                    qw[0, 2] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                    qw[1, 2] = last_q5
                    c = cos(last_q5 + DH[5, 0]); s = sin(last_q5 + DH[5, 0])
                    qw[1, 0] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                else:
                    qw[0, 2] = last_q5
                    c = cos(last_q5 + DH[5, 0]); s = sin(last_q5 + DH[5, 0])
                    qw[0, 0] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                    qw[1, 0] = last_q3
                    c = cos(last_q3 + DH[3, 0]); s = sin(last_q3 + DH[3, 0])
                    qw[1, 2] = atan2(c*R36[1,0] - s*R36[0,0], c*R36[0,0] + s*R36[1,0])
                solFlag[4*ind_arm + 2*ind_elbow + 0][2] = -1
                solFlag[4*ind_arm + 2*ind_elbow + 1][2] = -1
            else:
                cosqw = R36[2, 2]
                if ind_arm == 0:
                    qw[0, 1] = acos(cosqw); qw[1, 1] = -acos(cosqw)
                    qw[0, 0] = atan2(R36[1,2], R36[0,2])
                    qw[1, 0] = atan2(-R36[1,2], -R36[0,2])
                    qw[0, 2] = atan2(R36[2,1], -R36[2,0])
                    qw[1, 2] = atan2(-R36[2,1], R36[2,0])
                else:
                    qw[0, 1] = -acos(cosqw); qw[1, 1] = acos(cosqw)
                    qw[0, 0] = atan2(-R36[1,2], -R36[0,2])
                    qw[1, 0] = atan2(R36[1,2], R36[0,2])
                    qw[0, 2] = atan2(-R36[2,1], R36[2,0])
                    qw[1, 2] = atan2(R36[2,1], -R36[2,0])
                solFlag[4*ind_arm + 2*ind_elbow + 0][2] = 1
                solFlag[4*ind_arm + 2*ind_elbow + 1][2] = 1

            for ind_wrist in range(2):
                sol_idx = 4*ind_arm + 2*ind_elbow + ind_wrist
                solutions[sol_idx] = [
                    np.rad2deg(qs[ind_arm]),
                    np.rad2deg(qa[ind_elbow, 0]),
                    np.rad2deg(qa[ind_elbow, 1]),
                    np.rad2deg(qw[ind_wrist, 0]),
                    np.rad2deg(qw[ind_wrist, 1]),
                    np.rad2deg(qw[ind_wrist, 2]),
                ]
                if solFlag[sol_idx][0] == 0 or solFlag[sol_idx][1] == 0:
                    invalid_mask |= (1 << sol_idx)

    return solutions, invalid_mask

def map_joint_limits(solutions, invalid_mask):
    for i in range(8):
        if invalid_mask & (1 << i):
            continue
        for j in range(6):
            angle = solutions[i, j]
            min_a, max_a = JOINT_LIMITS[j]
            if angle < min_a and (min_a - angle) < KIN_LIMIT_TOLERANCE:
                angle = min_a
            if angle > max_a and (angle - max_a) < KIN_LIMIT_TOLERANCE:
                angle = max_a
            if angle < min_a or angle > max_a:
                a360 = angle + 360
                a_360 = angle - 360
                if a360 >= min_a - KIN_LIMIT_TOLERANCE and a360 <= max_a + KIN_LIMIT_TOLERANCE:
                    angle = max(min_a, min(max_a, a360))
                elif a_360 >= min_a - KIN_LIMIT_TOLERANCE and a_360 <= max_a + KIN_LIMIT_TOLERANCE:
                    angle = max(min_a, min(max_a, a_360))
                else:
                    invalid_mask |= (1 << i)
            solutions[i, j] = angle
    return solutions, invalid_mask

def get_optimal(solutions, invalid_mask, current_deg):
    best_idx, best_cost = -1, 1e9
    for i in range(8):
        if invalid_mask & (1 << i):
            continue
        cost = 0
        for j in range(6):
            d = solutions[i, j] - current_deg[j]
            while d > 180: d -= 360
            while d < -180: d += 360
            cost += abs(d) * JOINT_WEIGHTS[j]
        if cost < best_cost:
            best_cost = cost
            best_idx = i
    return best_idx, best_cost

def full_ik(T_target_mm, current_deg):
    sols, mask = solve_ik(T_target_mm, current_deg)
    sols, mask = map_joint_limits(sols, mask)
    idx, cost = get_optimal(sols, mask, current_deg)
    return sols, mask, idx, cost

# ========== 测试 ==========

def print_sep(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

def test_fk_rest():
    print_sep("TEST 1: FK(REST_POSE) vs T_0_6_reset")
    T = fk(REST_POSE)
    pos_mm = T[:3, 3] * 1000
    R = T[:3, :3]

    T_reset = np.array([
        [-0.2924, 0, 0.9563, 89.4],
        [0, 1, 0, 0],
        [-0.9563, 0, -0.2924, 146.7],
        [0, 0, 0, 1],
    ])

    print(f"  FK position (mm):  [{pos_mm[0]:.2f}, {pos_mm[1]:.2f}, {pos_mm[2]:.2f}]")
    print(f"  T_0_6_reset (mm):  [{T_reset[0,3]:.2f}, {T_reset[1,3]:.2f}, {T_reset[2,3]:.2f}]")
    pos_err = np.linalg.norm(pos_mm - T_reset[:3, 3])
    print(f"  Position error:    {pos_err:.4f} mm")

    print(f"\n  FK rotation:")
    for row in R:
        print(f"    [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}]")
    print(f"  T_0_6_reset rotation:")
    for row in T_reset[:3, :3]:
        print(f"    [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}]")
    rot_err = np.linalg.norm(R - T_reset[:3, :3])
    print(f"  Rotation error:    {rot_err:.6f}")

    ok = pos_err < 1.0 and rot_err < 0.01
    print(f"\n  RESULT: {'PASS' if ok else 'FAIL'}")
    return ok

def test_ik_roundtrip():
    print_sep("TEST 2: IK(FK(REST_POSE)) roundtrip")
    T = fk(REST_POSE)
    T_mm = T.copy()
    T_mm[:3, 3] *= 1000

    sols, mask, idx, cost = full_ik(T_mm, REST_POSE)

    print(f"  Valid solutions:")
    for i in range(8):
        valid = "* " if not (mask & (1<<i)) else "  "
        s = sols[i]
        tag = " <-- OPTIMAL" if i == idx else ""
        print(f"    {valid}Sol[{i}]: [{s[0]:7.2f}, {s[1]:7.2f}, {s[2]:7.2f}, {s[3]:7.2f}, {s[4]:7.2f}, {s[5]:7.2f}]{tag}")

    if idx >= 0:
        result = sols[idx]
        print(f"\n  IK result:   [{result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f}, {result[3]:.2f}, {result[4]:.2f}, {result[5]:.2f}]")
        print(f"  REST_POSE:   [{REST_POSE[0]:.2f}, {REST_POSE[1]:.2f}, {REST_POSE[2]:.2f}, {REST_POSE[3]:.2f}, {REST_POSE[4]:.2f}, {REST_POSE[5]:.2f}]")

        T_verify = fk(result)
        pos_verify = T_verify[:3, 3] * 1000
        pos_target = T_mm[:3, 3]
        err = np.linalg.norm(pos_verify - pos_target)
        print(f"\n  FK(IK) pos:  [{pos_verify[0]:.2f}, {pos_verify[1]:.2f}, {pos_verify[2]:.2f}]")
        print(f"  Target pos:  [{pos_target[0]:.2f}, {pos_target[1]:.2f}, {pos_target[2]:.2f}]")
        print(f"  Error:       {err:.4f} mm")
        ok = err < 1.0
    else:
        print(f"\n  IK FAILED - no valid solution!")
        ok = False

    print(f"\n  RESULT: {'PASS' if ok else 'FAIL'}")
    return ok

def test_offset_direction():
    print_sep("TEST 3: Offset direction analysis")
    T_rest = fk(REST_POSE)
    rest_pos = T_rest[:3, 3] * 1000

    offsets = [(0,0,0), (0,0,10), (10,0,0), (0,10,0), (0,0,-20)]

    print(f"  REST position (mm): [{rest_pos[0]:.1f}, {rest_pos[1]:.1f}, {rest_pos[2]:.1f}]")
    print(f"  REST rotation R06:")
    for row in T_rest[:3,:3]:
        print(f"    [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}]")
    print()
    print(f"  {'Offset':>15s}  |  {'Right-mul (end-effector)':>28s}  |  {'Left-mul (world)':>25s}  |  {'Direct add (world)':>25s}")
    print(f"  {'-'*15}  |  {'-'*28}  |  {'-'*25}  |  {'-'*25}")

    for dx, dy, dz in offsets:
        T_off = np.eye(4)
        T_off[0,3] = dx/1000; T_off[1,3] = dy/1000; T_off[2,3] = dz/1000

        # 方法A: T_reset * T_offset (C代码当前方式)
        T_a = T_rest @ T_off
        pos_a = T_a[:3, 3] * 1000

        # 方法B: 直接加偏移 (世界坐标系)
        pos_b = rest_pos + np.array([dx, dy, dz], dtype=float)

        # 方法C: T_offset * T_reset (左乘)
        T_c = T_off @ T_rest
        pos_c = T_c[:3, 3] * 1000

        label = f"({dx},{dy},{dz})"
        print(f"  {label:>15s}  |  [{pos_a[0]:7.1f},{pos_a[1]:7.1f},{pos_a[2]:7.1f}]  |"
              f"  [{pos_c[0]:7.1f},{pos_c[1]:7.1f},{pos_c[2]:7.1f}]  |"
              f"  [{pos_b[0]:7.1f},{pos_b[1]:7.1f},{pos_b[2]:7.1f}]")

    print(f"\n  C code uses RIGHT-multiply → offset in end-effector frame")
    print(f"  For intuitive world-frame: use direct position add (preserve orientation)")
    return True

def test_ik_world_offset():
    """测试4: 世界坐标系偏移的IK"""
    print_sep("TEST 4: IK with WORLD-FRAME offsets (direct add)")
    T_rest = fk(REST_POSE)
    rest_pos = T_rest[:3, 3] * 1000

    test_cases = [
        ("x 0 0 0",    0,  0,  0),
        ("x 0 0 10",   0,  0, 10),
        ("x 0 0 -10",  0,  0,-10),
        ("x 10 0 0",  10,  0,  0),
        ("x -10 0 0",-10,  0,  0),
        ("x 0 10 0",   0, 10,  0),
        ("x 0 -10 0",  0,-10,  0),
        ("x 10 10 10",10, 10, 10),
    ]

    current = REST_POSE.copy()
    all_pass = True

    for label, dx, dy, dz in test_cases:
        # 世界坐标系偏移: 只修改位置, 保持旋转
        T_target = T_rest.copy()
        T_target[0, 3] += dx / 1000.0
        T_target[1, 3] += dy / 1000.0
        T_target[2, 3] += dz / 1000.0
        T_target_mm = T_target.copy()
        T_target_mm[:3, 3] *= 1000

        target_pos = T_target_mm[:3, 3]
        sols, mask, idx, cost = full_ik(T_target_mm, current)

        if idx >= 0:
            result = sols[idx]
            T_verify = fk(result)
            pos_verify = T_verify[:3, 3] * 1000
            err = np.linalg.norm(pos_verify - target_pos)
            ok = err < 1.0
            print(f"  {label:>15s}: target=[{target_pos[0]:6.1f},{target_pos[1]:6.1f},{target_pos[2]:6.1f}]"
                  f"  IK=[{result[0]:6.1f},{result[1]:6.1f},{result[2]:6.1f},{result[3]:6.1f},{result[4]:6.1f},{result[5]:6.1f}]"
                  f"  err={err:.3f}mm {'PASS' if ok else 'FAIL'}")
        else:
            ok = False
            print(f"  {label:>15s}: target=[{target_pos[0]:6.1f},{target_pos[1]:6.1f},{target_pos[2]:6.1f}]"
                  f"  NO SOLUTION  FAIL")
        if not ok:
            all_pass = False

    print(f"\n  RESULT: {'ALL PASS' if all_pass else 'SOME FAILED'}")
    return all_pass

def test_c_code_offset():
    """测试5: 模拟C代码的右乘偏移方式"""
    print_sep("TEST 5: C code right-multiply T_reset * T_offset")

    # 使用C代码中的T_0_6_reset常量
    T_reset_c = np.array([
        [-0.2924, 0, 0.9563, 89.4/1000],
        [0, 1, 0, 0],
        [-0.9563, 0, -0.2924, 146.7/1000],
        [0, 0, 0, 1],
    ])

    offsets = [(0,0,0), (0,0,10), (10,0,0), (0,10,0)]
    current = REST_POSE.copy()
    all_pass = True

    for dx, dy, dz in offsets:
        T_off = np.eye(4)
        T_off[0,3] = dx/1000; T_off[1,3] = dy/1000; T_off[2,3] = dz/1000
        T_target_c = T_reset_c @ T_off
        T_target_c_mm = T_target_c.copy()
        T_target_c_mm[:3, 3] *= 1000
        target_pos = T_target_c_mm[:3, 3]

        sols, mask, idx, cost = full_ik(T_target_c_mm, current)
        label = f"({dx},{dy},{dz})"

        if idx >= 0:
            result = sols[idx]
            T_v = fk(result)
            pos_v = T_v[:3, 3] * 1000
            err = np.linalg.norm(pos_v - target_pos)
            ok = err < 1.0
            print(f"  {label:>12s}: target=[{target_pos[0]:6.1f},{target_pos[1]:6.1f},{target_pos[2]:6.1f}]"
                  f"  IK=[{result[0]:6.1f},{result[1]:6.1f},{result[2]:6.1f},{result[3]:6.1f},{result[4]:6.1f},{result[5]:6.1f}]"
                  f"  err={err:.3f}mm {'PASS' if ok else 'FAIL'}")
        else:
            ok = False
            print(f"  {label:>12s}: target=[{target_pos[0]:6.1f},{target_pos[1]:6.1f},{target_pos[2]:6.1f}]  NO SOLUTION  FAIL")
            for i in range(8):
                v = "* " if not (mask & (1<<i)) else "  "
                s = sols[i]
                print(f"              {v}Sol[{i}]: [{s[0]:7.2f}, {s[1]:7.2f}, {s[2]:7.2f}, {s[3]:7.2f}, {s[4]:7.2f}, {s[5]:7.2f}]")
        if not ok:
            all_pass = False

    print(f"\n  RESULT: {'ALL PASS' if all_pass else 'SOME FAILED'}")
    return all_pass

# ========== Main ==========
if __name__ == "__main__":
    print("dummy-auk IK/FK Verification (v2 - link vector FK)")
    print("=" * 60)

    results = []
    results.append(("FK REST_POSE",          test_fk_rest()))
    results.append(("IK roundtrip",          test_ik_roundtrip()))
    results.append(("Offset direction",      test_offset_direction()))
    results.append(("IK world offsets",      test_ik_world_offset()))
    results.append(("IK C-code offsets",     test_c_code_offset()))

    print_sep("SUMMARY")
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print()
    if all(r[1] for r in results):
        print("  All tests passed!")
    else:
        print("  Some tests FAILED - review output above")
