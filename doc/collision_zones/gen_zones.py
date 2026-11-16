#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""为 9 种运动状态生成 collision_monitor 三级避障边界示意图 (SVG)，纯标准库。

索引约定（全文件统一，最容易搞反的地方）：
    state["layers"][tier]  ->  (L3, L2, L1)   三元素，由内到外
    state["layers"][tier][lv] :  lv=0 -> L3 停止(最内)
                                 lv=1 -> L2 减速(中间)
                                 lv=2 -> L1 预警(最外)
    颜色           : L1 绿 #43a047 / L2 橙 #fb8c00 / L3 红 #e53935
"""

import html
import math
import os

OUT = os.path.dirname(os.path.abspath(__file__))

# ---------------- 尺寸档位 ----------------
# sim ：与仿真模型 myworld_bringup/models/myworld2/models/diffbot/model.sdf 一致（0.52 × 0.36 m）
# real：实车 footprint [[±0.6, ±0.4]]（1.2 × 0.8 m）；载货外廓取 max(货框, 车体)
PROFILES = {
    "sim": dict(R_F=0.26, R_B=0.26, R_S=0.18, C_F=0.60, C_B=0.35, C_S=0.35),
    "real": dict(R_F=0.60, R_B=0.60, R_S=0.40, C_F=0.60, C_B=0.60, C_S=0.40),
}

# ---------------- 默认（仿真）尺寸 ----------------
_SIM = PROFILES["sim"]
R_F, R_B, R_S = _SIM["R_F"], _SIM["R_B"], _SIM["R_S"]   # 车体半长(前) / 半长(后) / 半宽
C_F, C_B, C_S = _SIM["C_F"], _SIM["C_B"], _SIM["C_S"]   # 货框/载货态外廓: 前行 / 后行 / 半宽
CARGO_R = math.hypot(C_F, C_S)     # 货框外接半径 0.694
BODY_R = math.hypot(R_F, R_S)      # 车体半对角线 0.316


# ---------------- 净空（clearance）唯一出处 ----------------
# 全文件的距离数字都从这里取：净空 = 边界 − 机器人外缘（载货时算到货框外缘）。
EDGE_IDLE = (R_F, R_B, R_S)        # 空载：车体 行进向 / 反向 / 侧向 外缘
EDGE_CARGO = (C_F, C_B, C_S)       # 载货：货框外缘（比车体大）


def fmt(v):
    return f"{v:.2f}"


def cl_rect(layers, tier, lv, edge=None):
    """矩形状态净空三元组 (行进向, 反向, 侧向)。layers 结构与 state["layers"] 相同。"""
    if edge is None:          # 调用时取当前档位的基准，不能在 def 时绑定
        edge = EDGE_IDLE
    return tuple(layers[tier][lv][i] - edge[i] for i in range(3))


def cl_rect_str(layers, tier, lv, edge=None):
    """矩形状态净空三元组拼成 "0.30 / 0.15 / 0.10" """
    return " / ".join(fmt(v) for v in cl_rect(layers, tier, lv, edge))


def cl_circle(layers, tier, lv, edge=None):
    """圆形状态净空（标量）"""
    if edge is None:          # 调用时取当前档位的车体半对角线
        edge = BODY_R
    return layers[tier][lv] - edge


def robot_edge(state, axis):
    """该方向上机器人本体（载货时含货框）最外缘到中心的距离。
    axis: 0=行进向 1=反向 2=侧向；圆形状态用车体半对角线 / 货框外接半径。"""
    if state["kind"] == "circle":
        return CARGO_R if state.get("cargo") else BODY_R
    return (C_F, C_B, C_S)[axis] if state.get("cargo") else (R_F, R_B, R_S)[axis]


def clearance(state, tier, lv, axis):
    """边界到机器人外缘的净空（m）。可为 0（边界与货框齐平）；屏蔽方向不适用。"""
    if state["kind"] == "circle":
        return state["layers"][tier][lv] - robot_edge(state, axis)
    return state["layers"][tier][lv][axis] - robot_edge(state, axis)


def cl(state, tier, lv, axis):
    """净空数值串"""
    return fmt(clearance(state, tier, lv, axis))


def cl_tri(state, tier, lv):
    """矩形状态：行进向 / 反向 / 侧向 三个净空拼成 "0.30 / 0.15 / 0.10" """
    return " / ".join(cl(state, tier, lv, i) for i in range(3))

# ---------------- 分层间隙律 ----------------
_dF = (0.0, 0.15, 0.30, 0.50)
_dB = (0.0, 0.03, 0.06, 0.10)
_dS = (0.0, 0.03, 0.06, 0.10)


def _front(F0, k):
    g1, g2 = 0.50 - 0.033 * k, 0.70 - 0.033 * k
    a = F0 + _dF[k]
    return a, a + g1, a + g1 + g2


def _rear(B0, k):
    g1, g2 = 0.15 - 0.017 * k, 0.30 - 0.017 * k
    a = B0 + _dB[k]
    return a, a + g1, a + g1 + g2


def _side(S0, k):
    g1, g2 = 0.10 - 0.007 * k, 0.15 - 0.010 * k
    a = S0 + _dS[k]
    return a, a + g1, a + g1 + g2


def rect_layers(F0, B0, S0, tiers=4):
    """-> [tier][lv] 每层 (F, B, S)，lv 序 L3, L2, L1"""
    return [tuple(zip(_front(F0, k), _rear(B0, k), _side(S0, k))) for k in range(tiers)]


def micro_layers(F0, B0, S0):
    """抓料/微小场景：间隙比主律更小，预警层不外扩，单档"""
    l3 = (F0, B0, S0)
    l2 = (F0 + 0.08, B0 + 0.05, S0 + 0.05)
    l1 = (F0 + 0.20, B0 + 0.14, S0 + 0.12)
    return [(l3, l2, l1)]


def circle_layers(R3, n=1):
    """n=1 单档; n>1 按档给不同基准半径"""
    return [(r, r + 0.10, r + 0.25) for r in ([R3] if n == 1 else R3)]


def kill_front(layers):
    """把前向压到 0（屏蔽前方）"""
    return [tuple((0.0, b, s) for (_f, b, s) in t) for t in layers]


def kill_rear(layers):
    """把后向压到 0（屏蔽后方）"""
    return [tuple((f, 0.0, s) for (f, _b, s) in t) for t in layers]


# ---------------- 9 个状态 ----------------
T_V4 = ("0.0–0.3", "0.3–0.6", "0.6–1.0", "1.0–3.0")
T_W3 = ("0.5–1.0", "1.0–2.0", "2.0–5.0")

IDLE_F_L = rect_layers(0.56, 0.41, 0.28)
CARGO_L = rect_layers(0.80, 0.35, 0.45)
GRAB_L = micro_layers(0.38, 0.30, 0.22)

# 净空数值串，供各状态 notes 引用（空载行走 / 载货行走 / 抓料 / 圆环）
_I3v, _I2v, _I1v = (cl_rect(IDLE_F_L, 0, lv) for lv in range(3))
_C3v, _C2v, _C1v = (cl_rect(CARGO_L, 0, lv, EDGE_CARGO) for lv in range(3))
_G3v, _G1v = cl_rect(GRAB_L, 0, 0), cl_rect(GRAB_L, 0, 2)


def s3(vals):
    """净空三元组 -> "0.30 / 0.15 / 0.10" """
    return " / ".join(fmt(v) for v in vals)


_I3s, _I2s, _I1s = s3(_I3v), s3(_I2v), s3(_I1v)
_G3s = s3(_G3v)
_IK0, _IK3 = cl_rect(IDLE_F_L, 0, 0)[0], cl_rect(IDLE_F_L, 3, 0)[0]   # 空载行进向 L3 首/末档
_CK0 = cl_rect(CARGO_L, 0, 0, EDGE_CARGO)[0]
_SPIN = circle_layers((0.45, 0.55, 0.70), n=3)
_SPv = tuple(cl_circle(_SPIN, t, 2) for t in range(3))               # 旋转 L1 各档净空
_SP3 = tuple(cl_circle(_SPIN, t, 0) for t in range(3))               # 旋转 L3 各档净空

STATES = [
    dict(no=1, name="空载正走", key="idle_forward", dir=1, kind="rect", axis="v  (m/s)",
         tiers=T_V4, layers=IDLE_F_L, cargo=None,
         notes=[
             f"L1 预警层（第 0 档）净空 {_I1s} m（行进向 / 反向 / 侧向）—— 行进向放得最远，"
             f"指“车头外缘再往前 {fmt(_I1v[0])} m 就开始减速”",
             f"三层由内到外：L3 停止 {_I3s} → L2 减速 {_I2s} → L1 预警 {_I1s} m"
             f"（行进向 / 反向 / 侧向，均为自车体外缘量起的净空）",
             f"外扩量：行进向 L1−L3 = {fmt(_I1v[0] - _I3v[0])} m，"
             f"侧向仅 {fmt(_I1v[2] - _I3v[2])} m —— "
             f"正前是高速接近的主要风险方向，侧向过宽会在窄通道里频繁误触发",
         ]),
    dict(no=2, name="空载倒走", key="idle_backward", dir=-1, kind="rect", axis="v  (m/s)",
         tiers=T_V4, layers=IDLE_F_L, cargo=None,
         notes=[
             "与空载正走完全镜像：把“前方”定义改到 -x，数值一字不改",
             f"L3 净空：行进向（车尾侧）{fmt(_I3v[0])} / 反向（车头侧）{fmt(_I3v[1])}"
             f" / 侧向 {fmt(_I3v[2])} m",
             "实现上只需把矩形 x 取负：[[-F,-S],[-F,S],[B,S],[B,-S]]",
         ]),
    dict(no=3, name="正走抓料框", key="grab_approach", dir=1, kind="rect", axis="—",
         tiers=("固定",), layers=GRAB_L, cargo=None,
         notes=[
             f"用矩形而非圆：圆要包住车体至少得取 R = {fmt(BODY_R)} m（车体半对角线），"
             f"而要给出行进向 {fmt(_G3v[0])} m 净空就得取 R = {fmt(R_F + _G3v[0])} m，"
             f"侧向同时被撑到 {fmt(R_F + _G3v[0])} m；矩形则能只放行进向、把侧向压在 "
             f"{fmt(_G3v[2])} m，正对“确保能靠近货物”",
             f"三级整体收缩：L3 净空 {_G3s} m（行进向 / 反向 / 侧向），"
             f"行进向净空仅 {fmt(_G3v[0])} m —— 车头外缘再往前 {fmt(_G3v[0])} m 就到停止边界，"
             f"刚好够完成抓取",
             "不做速度分档：对位阶段车速由料框对位逻辑压住，不需要靠边界降速",
             f"若实际存在较大朝向偏差（斜向进场、需在料框旁原地转头），矩形侧向净空只有 "
             f"{fmt(_G3v[2])} m，斜向进场时这点余量不够用；这种情况应改回圆，"
             f"取 R = 0.40 m（净空 {fmt(0.40 - BODY_R)} m）",
         ]),
    dict(no=4, name="搬货倒走", key="cargo_backward", dir=-1, kind="rect", axis="v  (m/s)",
         tiers=T_V4, layers=CARGO_L, cargo="travel",
         notes=[
             "货框外廓 1.20 × 0.70 m（相对车体中心 前 0.60 / 后 0.35 / 半宽 0.35）",
             f"本状态的净空一律自货框外缘量起，不是自车体。L3 行进向净空 {fmt(_C3v[0])} m，"
             f"即停止边界比货框前缘还远出 {fmt(_C3v[0])} m，确保停止层始终包住货物",
             f"反向净空 {fmt(_C3v[1])} m、侧向净空 {fmt(_C3v[2])} m 收紧 —— "
             f"避免转向贴边时被车头侧误触发",
             f"注意反向 L3 净空为 0：边界与货框后缘齐平（{fmt(C_B)} m）。若货框改为居中"
             f"（后缘也到 {fmt(C_F)} m），则反向 L3 边界须跟着货框同步外扩到 {fmt(C_F)} m"
             f"（即再外扩 {fmt(C_F - C_B)} m），否则倒车时货框尾部无保护",
         ]),
    dict(no=5, name="搬货正走", key="cargo_forward", dir=1, kind="rect", axis="v  (m/s)",
         tiers=T_V4, layers=CARGO_L, cargo="travel",
         notes=[
             "与搬货倒走完全镜像：行进方向一侧覆盖货物，另一侧与两侧收紧",
             f"L3 净空 {s3(_C3v)} → L2 净空 {s3(_C2v)} → L1 净空 {s3(_C1v)} m"
             f"（行进向 / 反向 / 侧向，自货框外缘量起）",
             f"L1 相对 L3 的外扩量：行进向 +{fmt(_C1v[0] - _C3v[0])} m、"
             f"侧向 +{fmt(_C1v[2] - _C3v[2])} m（均自货框外缘量起）",
             f"速度分档沿用主律：行进向 L3 净空由 {fmt(_CK0)} m（第 0 档）随速度放宽，"
             f"侧向、反向只加 0.03~0.04 m/档",
         ]),
    dict(no=6, name="静止", key="stationary", dir=0, kind="circle", axis="—",
         tiers=("固定",), layers=circle_layers(0.35), cargo=None,
         notes=[
             f"同心圆 R = 0.35 / 0.45 / 0.60 m，对应净空 {fmt(cl_circle(circle_layers(0.35), 0, 0))}"
             f" / {fmt(cl_circle(circle_layers(0.35), 0, 1))} / {fmt(cl_circle(circle_layers(0.35), 0, 2))} m"
             f"（半径 − 车体半对角线 {fmt(BODY_R)} m）",
             f"L3 半径 0.35 m 只比车体半对角线 {fmt(BODY_R)} m 大 3 cm，净空因此仅 "
             f"{fmt(cl_circle(circle_layers(0.35), 0, 0))} m",
             "静止时车不动，误触发只发零速、代价极低，因此可以取到最小",
             "作用：防被外部移动物体/人员顶到，也防静止中被残留的微小指令带偏",
         ]),
    dict(no=7, name="原地旋转", key="spin_in_place", dir=0, kind="circle", axis="|w|  (rad/s)",
         tiers=T_W3, layers=circle_layers((0.45, 0.55, 0.70), n=3), cargo=None,
         notes=[
             f"三级同心圆，L3 半径随 |w| 分 3 档：0.45 / 0.55 / 0.70 m，"
             f"净空分别为 {s3(_SP3)} m（半径 − 车体半对角线 {fmt(BODY_R)} m）",
             "旋转时车体扫过的是一整圈，所以用圆而非矩形；四角也需要等距保护",
             f"载货旋转时 L3 半径须 ≥ 货框外接半径 √(0.60²+0.35²) = {fmt(CARGO_R)} m，"
             f"即至少取 0.70 m 档（此时自货框外缘量起的净空仅 {fmt(0.70 - CARGO_R)} m）",
         ]),
    dict(no=8, name="去充电", key="dock_charge", dir=1, kind="rect", axis="v  (m/s)",
         tiers=T_V4, layers=kill_front(IDLE_F_L), cargo=None, mask="front",
         notes=[
             "前方 L3/L2/L1 前缘全部压在 x = 0（车体中心），x > 0 区域完全不参与碰撞判定；"
             "该方向没有净空可言，图中记作「0（屏蔽）」",
             f"后方（反向）净空 {fmt(_I3v[1])} / {fmt(_I2v[1])} / {fmt(_I1v[1])} m、"
             f"侧向净空 {fmt(_I3v[2])} / {fmt(_I2v[2])} / {fmt(_I1v[2])} m（L3 / L2 / L1），"
             f"沿用空载正走值并随速度分档",
             f"代价：车体前 {fmt(R_F)} m 的侧向也失去保护；若需保留，建议用 exclusion_zone "
             f"只挖掉充电桩本体，而不是整体屏蔽前方",
         ]),
    dict(no=9, name="充电结束离开", key="undock_leave", dir=-1, kind="rect", axis="v  (m/s)",
         tiers=T_V4, layers=kill_rear(IDLE_F_L), cargo=None, mask="rear",
         notes=[
             "与去充电衔接：去充电是车头 +x 方向扎入充电桩，故离开时车头不动、沿 -x 倒车退出",
             "「后方」= 行进方向的反侧 = 充电桩所在侧（+x），该半平面屏蔽，净空记作「0（屏蔽）」；"
             f"行进向仍保留净空 {fmt(_I3v[0])} / {fmt(_I2v[0])} / {fmt(_I1v[0])} m"
             f"（L3 / L2 / L1）",
             "若现场实际是「调头后向前驶离」，则屏蔽侧应换成 -x，本图整体左右镜像",
         ]),
]

# ---------------- 尺寸档切换 ----------------
# 导入时按仿真档建好 STATES；实时切换只重建几何，notes 文字仍描述仿真档（仅用于 SVG）。
_BASE_STATE_LAYERS = {st["no"]: st["layers"] for st in STATES}
CURRENT_PROFILE = "sim"


def apply_profile(name):
    """按尺寸档重建 STATES 的 layers 与净空基准。

    sim : 0.52 × 0.36 m 仿真车体，全部数值与旧版逐点一致。
    real: 1.2 × 0.8 m 实车外廓；边界 = 实测外缘 + 原设计净空。
    """
    global R_F, R_B, R_S, C_F, C_B, C_S
    global CARGO_R, BODY_R, EDGE_IDLE, EDGE_CARGO, CURRENT_PROFILE, _SPIN

    if name not in PROFILES:
        raise ValueError(f"未知尺寸档 {name!r}，可选：{', '.join(PROFILES)}")

    dims = PROFILES[name]
    R_F, R_B, R_S = dims["R_F"], dims["R_B"], dims["R_S"]
    C_F, C_B, C_S = dims["C_F"], dims["C_B"], dims["C_S"]
    CARGO_R = math.hypot(C_F, C_S)
    BODY_R = math.hypot(R_F, R_S)
    EDGE_IDLE = (R_F, R_B, R_S)
    EDGE_CARGO = (C_F, C_B, C_S)

    if name == "sim":
        # 恢复导入时的对象，保证仿真行为与尺寸逐点不变。
        for st in STATES:
            st["layers"] = _BASE_STATE_LAYERS[st["no"]]
        _SPIN = _BASE_STATE_LAYERS[7]
    else:
        idle = rect_layers(R_F + 0.30, R_B + 0.15, R_S + 0.10)
        cargo = rect_layers(C_F + 0.20, C_B + 0.00, C_S + 0.10)
        grab = micro_layers(R_F + 0.12, R_B + 0.04, R_S + 0.04)
        static = circle_layers(BODY_R + 0.03)
        spin = circle_layers((BODY_R + 0.13, BODY_R + 0.23, BODY_R + 0.38), n=3)
        layers = {1: idle, 2: idle, 3: grab, 4: cargo, 5: cargo,
                  6: static, 7: spin,
                  8: kill_front(idle), 9: kill_rear(idle)}
        for st in STATES:
            st["layers"] = layers[st["no"]]
        _SPIN = spin

    CURRENT_PROFILE = name


# ---------------- 绘图基础设施 ----------------
C_L1, C_L2, C_L3 = "#43a047", "#fb8c00", "#e53935"
# 填充索引与 layers[tier] 一致：0=L3(最内,填充最重) 1=L2 2=L1(最外,填充最轻)
FILL_OP = (0.20, 0.13, 0.09)
# (索引, 名称, 颜色) —— 索引即 layers[tier] 的层号，绘制与图例都以此为准
NAME_OF_LV = ((0, "L3 停止", C_L3), (1, "L2 减速", C_L2), (2, "L1 预警", C_L1))
TIER_COLORS = ("#1e88e5", "#8e24aa", "#e53935", "#00897b")
FONT = "Noto Sans CJK SC, Noto Sans CJK JP, Source Han Sans SC, sans-serif"


def esc(s):
    return html.escape(str(s), quote=True)


def lvname(lv):
    return NAME_OF_LV[lv][1]


class View:
    def __init__(self, W, H, xmin, xmax, ymin, ymax, pad=34):
        aw, ah = W - 2 * pad, H - 2 * pad
        self.s = min(aw / (xmax - xmin), ah / (ymax - ymin))
        self.cx = pad + (aw - (xmax - xmin) * self.s) / 2 - xmin * self.s
        self.cy = pad + (ah - (ymax - ymin) * self.s) / 2 + ymax * self.s
        self.W, self.H = W, H

    def X(self, x):
        return self.cx + x * self.s

    def Y(self, y):
        return self.cy - y * self.s

    def P(self, pts):
        return " ".join(f"{self.X(x):.1f},{self.Y(y):.1f}" for x, y in pts)


def rect_pts(F, B, S, d):
    xmax, xmin = (F, -B) if d > 0 else (B, -F)
    return [(xmax, -S), (xmax, S), (xmin, S), (xmin, -S)]


def circle_pts(R, n=120):
    return [(R * math.cos(2 * math.pi * i / n), R * math.sin(2 * math.pi * i / n)) for i in range(n)]


def shape_pts(state, lv, tier):
    """lv: 0=L3 1=L2 2=L1"""
    if state["kind"] == "circle":
        return circle_pts(state["layers"][tier][lv])
    F, B, S = state["layers"][tier][lv]
    return rect_pts(F, B, S, state["dir"])


def extent(state, tier=0):
    """该档 L1（最外层 lv=2）的 x 正向 / x 负向 / y 向跨度 —— 只用于定视图范围"""
    if state["kind"] == "circle":
        r = state["layers"][tier][2]
        return r, r, r
    F, B, S = state["layers"][tier][2]
    return (F, B, S) if state["dir"] > 0 else (B, F, S)


def extent_all(state):
    return max(max(extent(state, t)) for t in range(len(state["tiers"])))


def cargo_pts(state):
    if not state.get("cargo"):
        return None
    d = state["dir"]
    xmax, xmin = (C_F, -C_B) if d > 0 else (C_B, -C_F)
    return [(xmax, -C_S), (xmax, C_S), (xmin, C_S), (xmin, -C_S)]


def body_pts():
    return rect_pts(R_F, R_B, R_S, 1)


def robot_edge(state, axis):
    """该方向上机器人本体（载货时含货框）最外缘到中心的距离。
    axis: 0=行进向 1=反向 2=侧向；圆形状态用车体半对角线 / 货框外接半径。"""
    if state["kind"] == "circle":
        return CARGO_R if state.get("cargo") else BODY_R
    return (C_F, C_B, C_S)[axis] if state.get("cargo") else (R_F, R_B, R_S)[axis]


def draw_hatch_defs():
    return ('<defs><pattern id="mask" width="10" height="10" patternUnits="userSpaceOnUse" '
            'patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="10" '
            'stroke="#b0bec5" stroke-width="2.2"/></pattern></defs>')


def draw_masked_sector(buf, v, state, xmax, ymax):
    """在屏蔽方向上盖一层斜纹"""
    if not state.get("mask"):
        return
    xs = [p[0] for p in shape_pts(state, 0, 0)]      # L3 被压平到 x = 0
    side = 1 if max(xs) == 0 else -1
    if side > 0:
        pts = [(0, -ymax), (xmax, -ymax), (xmax, ymax), (0, ymax)]
    else:
        pts = [(-xmax, -ymax), (0, -ymax), (0, ymax), (-xmax, ymax)]
    buf.append(f'<polygon points="{v.P(pts)}" fill="url(#mask)" opacity="0.55" '
               f'stroke="#90a4ae" stroke-width="1.2" stroke-dasharray="5 4"/>')
    buf.append(f'<text x="{v.X(side * xmax * 0.55):.0f}" y="{v.Y(ymax * 0.72):.0f}" '
               f'text-anchor="middle" font-size="15" fill="#546e7a" font-weight="600">'
               f'屏蔽区（边界压到车体中心，不参与碰撞判定）</text>')


def draw_axes(buf, v, span):
    buf.append(f'<line x1="{v.X(-span):.0f}" y1="{v.Y(0):.0f}" x2="{v.X(span):.0f}" '
               f'y2="{v.Y(0):.0f}" stroke="#cfd8dc" stroke-width="1" stroke-dasharray="4 4"/>')
    buf.append(f'<line x1="{v.X(0):.0f}" y1="{v.Y(-span):.0f}" x2="{v.X(0):.0f}" '
               f'y2="{v.Y(span):.0f}" stroke="#cfd8dc" stroke-width="1" stroke-dasharray="4 4"/>')


def draw_layers(buf, v, state, tier):
    """由外到内画（L1 → L2 → L3）：让最里面的红色停止线永远压在最上层"""
    for lv in (2, 1, 0):
        col = NAME_OF_LV[lv][2]
        buf.append(f'<polygon points="{v.P(shape_pts(state, lv, tier))}" fill="{col}" '
                   f'fill-opacity="{FILL_OP[lv]}" stroke="{col}" stroke-width="2.2" '
                   f'stroke-linejoin="round"/>')


def draw_tier_outlines(buf, v, state, lv=2):
    """各档的同一层叠加对比（默认 L1 预警，lv=2）"""
    for t, lab in enumerate(state["tiers"]):
        col = TIER_COLORS[t % len(TIER_COLORS)]
        buf.append(f'<polygon points="{v.P(shape_pts(state, lv, t))}" fill="none" stroke="{col}" '
                   f'stroke-width="2.4" stroke-dasharray="8 4" stroke-linejoin="round"/>')


def draw_body_and_cargo(buf, v, state):
    cp = cargo_pts(state)
    if cp:
        buf.append(f'<polygon points="{v.P(cp)}" fill="#795548" fill-opacity="0.18" '
                   f'stroke="#6d4c41" stroke-width="2" stroke-dasharray="7 4"/>')
    buf.append(f'<polygon points="{v.P(body_pts())}" fill="#263238" fill-opacity="0.88" '
               f'stroke="#101418" stroke-width="1.6"/>')


def draw_arrow(buf, v, state, span):
    d = state["dir"]
    if d == 0:
        return
    y = v.Y(0)
    x0, x1 = v.X(d * span * 0.18), v.X(d * span * 0.62)
    buf.append(f'<line x1="{x0:.0f}" y1="{y:.0f}" x2="{x1:.0f}" y2="{y:.0f}" '
               f'stroke="#1565c0" stroke-width="3"/>')
    hd = 11
    tip = x1 if d > 0 else x0
    bx = x1 - hd * d if d > 0 else x0 + hd
    buf.append(f'<polygon points="{tip:.0f},{y:.0f} {bx:.0f},{y - hd * 0.55:.0f} '
               f'{bx:.0f},{y + hd * 0.55:.0f}" fill="#1565c0"/>')


DIM_COLORS = {0: "#c62828", 1: "#e65100", 2: "#2e7d32"}


def draw_dims(buf, v, state, tier=0, lv=2):
    """标注某一层（默认 L1 预警 lv=2）的净空。
    标注线自机器人本体最外缘画到边界，因此线的长度本身就是净空。"""
    col = DIM_COLORS[lv]
    y0, x0 = v.Y(0), v.X(0)
    if state["kind"] == "circle":
        R = state["layers"][tier][lv]
        e = robot_edge(state, 2)                       # 含货时用货框外接半径
        buf.append(f'<line x1="{x0:.0f}" y1="{y0:.0f}" x2="{v.X(R):.0f}" y2="{y0:.0f}" '
                   f'stroke="{col}" stroke-width="1.3" stroke-dasharray="3 3"/>')
        buf.append(f'<line x1="{v.X(e):.0f}" y1="{y0 - 6:.0f}" x2="{v.X(e):.0f}" '
                   f'y2="{y0 + 6:.0f}" stroke="{col}" stroke-width="1.6"/>')
        buf.append(f'<text x="{(x0 + v.X(R)) / 2:.0f}" y="{y0 - 10:.0f}" text-anchor="middle" '
                   f'font-size="14" fill="{col}" font-weight="600">R={fmt(R)}</text>')
        buf.append(f'<text x="{v.X(R) + 8:.0f}" y="{y0 + 22:.0f}" font-size="13.5" '
                   f'fill="{col}">净空 {fmt(R - e)}</text>')
        return
    F, B, S = state["layers"][tier][lv]
    d = state["dir"]
    # 行进向 / 反向：本体外缘 -> 边界。dir<0 时行进向在 -x，故轴的映射跟着翻。
    for val, ax, lab in ((F, 0, "行进向"), (B, 1, "反向")):
        if val == 0:                                   # 屏蔽方向不画零长标注线
            buf.append(f'<text x="{x0:.0f}" y="{y0 - 10:.0f}" text-anchor="middle" '
                       f'font-size="14" fill="{col}" font-weight="600">{lab} 0（屏蔽）</text>')
            continue
        e = robot_edge(state, ax)
        sgn = d if ax == 0 else -d                     # 该轴在屏幕上的正负
        x_border, x_edge = v.X(sgn * val), v.X(sgn * e)
        if ax == 1:                                    # 反向标注线画在下方，避开行进向文字
            buf.append(f'<line x1="{x_edge:.0f}" y1="{y0:.0f}" x2="{x_border:.0f}" y2="{y0:.0f}" '
                       f'stroke="{col}" stroke-width="1.3" stroke-dasharray="3 3"/>')
            buf.append(f'<line x1="{x_edge:.0f}" y1="{y0 - 6:.0f}" x2="{x_edge:.0f}" '
                       f'y2="{y0 + 6:.0f}" stroke="{col}" stroke-width="1.6"/>')
            buf.append(f'<text x="{(x_edge + x_border) / 2:.0f}" y="{y0 + 20:.0f}" '
                       f'text-anchor="middle" font-size="14" fill="{col}" font-weight="600">'
                       f'{lab} {fmt(val - e)}</text>')
        else:
            buf.append(f'<line x1="{x_edge:.0f}" y1="{y0:.0f}" x2="{x_border:.0f}" y2="{y0:.0f}" '
                       f'stroke="{col}" stroke-width="1.3" stroke-dasharray="3 3"/>')
            buf.append(f'<line x1="{x_edge:.0f}" y1="{y0 - 6:.0f}" x2="{x_edge:.0f}" '
                       f'y2="{y0 + 6:.0f}" stroke="{col}" stroke-width="1.6"/>')
            buf.append(f'<text x="{(x_edge + x_border) / 2:.0f}" y="{y0 - 10:.0f}" '
                       f'text-anchor="middle" font-size="14" fill="{col}" font-weight="600">'
                       f'{lab} {fmt(val - e)}</text>')
    # 侧向：本体外缘 -> 边界（画在 +y 侧）
    e = robot_edge(state, 2)
    buf.append(f'<line x1="{x0:.0f}" y1="{v.Y(e):.0f}" x2="{x0:.0f}" y2="{v.Y(S):.0f}" '
               f'stroke="{col}" stroke-width="1.3" stroke-dasharray="3 3"/>')
    buf.append(f'<line x1="{x0 - 6:.0f}" y1="{v.Y(e):.0f}" x2="{x0 + 6:.0f}" y2="{v.Y(e):.0f}" '
               f'stroke="{col}" stroke-width="1.6"/>')
    buf.append(f'<text x="{x0 + 10:.0f}" y="{(v.Y(e) + v.Y(S)) / 2:.0f}" font-size="14" '
               f'fill="{col}" font-weight="600">侧向 {fmt(S - e)}</text>')


def draw_scalebar(buf, v, x, y, meters=0.5):
    x2 = x + meters * v.s
    buf.append(f'<line x1="{x:.0f}" y1="{y:.0f}" x2="{x2:.0f}" y2="{y:.0f}" '
               f'stroke="#37474f" stroke-width="2.5"/>')
    for xx in (x, x2):
        buf.append(f'<line x1="{xx:.0f}" y1="{y - 5:.0f}" x2="{xx:.0f}" y2="{y + 5:.0f}" '
                   f'stroke="#37474f" stroke-width="2.5"/>')
    buf.append(f'<text x="{(x + x2) / 2:.0f}" y="{y + 20:.0f}" text-anchor="middle" '
               f'font-size="13" fill="#37474f">{meters} m</text>')


def draw_legend(buf, x, y, entries):
    for i, (col, lab, dash) in enumerate(entries):
        yy = y + i * 24
        da = f' stroke-dasharray="{dash}"' if dash else ""
        buf.append(f'<line x1="{x}" y1="{yy}" x2="{x + 30}" y2="{yy}" stroke="{col}" '
                   f'stroke-width="3"{da}/>')
        buf.append(f'<text x="{x + 40}" y="{yy + 5}" font-size="14" fill="#37474f">'
                   f'{esc(lab)}</text>')


def draw_legend_h(buf, x, y, entries, gap=170):
    for i, (col, lab) in enumerate(entries):
        xx = x + i * gap
        buf.append(f'<line x1="{xx}" y1="{y}" x2="{xx + 28}" y2="{y}" stroke="{col}" '
                   f'stroke-width="4"/>')
        buf.append(f'<text x="{xx + 36}" y="{y + 5}" font-size="14.5" fill="#37474f">'
                   f'{esc(lab)}</text>')


# ---------------- 边界距离表 ----------------
def _rng(vals):
    """档位范围：单档给一个值，多档给 第0档→最高档"""
    lo, hi = min(vals), max(vals)
    return fmt(lo) if abs(hi - lo) < 1e-9 else f"{fmt(lo)}→{fmt(hi)}"


def _series(state, lv, clear=False):
    """某层在全部档位下的序列；圆环形态只返回半径序列。
    clear=True 时返回净空（自机器人外缘量起），否则返回总尺寸（自中心量起）。"""
    if state["kind"] == "circle":
        return [clearance(state, t, lv, 2) if clear else state["layers"][t][lv]
                for t in range(len(state["layers"]))]
    return [tuple(clearance(state, t, lv, i) if clear else state["layers"][t][lv][i]
                  for t in range(len(state["layers"]))) for i in range(3)]


def draw_boundary_table(buf, state, x, y):
    """设计要点下方：列出 L3/L2/L1 三层相对机器人外缘的净空与范围（单位 m）"""
    circ = state["kind"] == "circle"
    # 第一列只放色块 + 名称，数值列从 x+200 起 —— 避免与名称重叠
    if circ:
        cols = (x, x + 200, x + 380)
        hdr = ("边界", "半径 R（自中心）", "净空（自外缘）")
    else:
        cols = (x, x + 200, x + 380, x + 560)
        hdr = ("边界", "行进向", "反向", "侧向")
    masked_axis = 0 if state.get("mask") == "front" else (1 if state.get("mask") == "rear" else -1)
    rule_end = cols[-1] + 150

    title = (f'三个边界框的半径与净空（m）' if circ
             else f'三个边界框的净空：边界到机器人外缘的距离（m）')
    buf.append(f'<text x="{x}" y="{y}" font-size="16" font-weight="600" fill="#1a237e">'
               f'{esc(title)}</text>')
    for cx, h in zip(cols, hdr):
        buf.append(f'<text x="{cx}" y="{y + 20}" font-size="13.5" font-weight="600" '
                   f'fill="#78909c">{esc(h)}</text>')
    buf.append(f'<line x1="{x}" y1="{y + 29}" x2="{rule_end}" y2="{y + 29}" '
               f'stroke="#cfd8dc" stroke-width="1.2"/>')

    for i, (lv, name, col) in enumerate(NAME_OF_LV):
        yy = y + 58 + i * 32
        buf.append(f'<rect x="{x}" y="{yy - 15}" width="{rule_end - x}" height="30" '
                   f'fill="{col}" fill-opacity="0.05"/>')
        buf.append(f'<rect x="{x}" y="{yy - 10}" width="14" height="14" rx="2" fill="{col}"/>')
        buf.append(f'<text x="{x + 22}" y="{yy}" font-size="15" font-weight="600" '
                   f'fill="{col}">{esc(name)}</text>')
        if circ:
            cells = [_rng(_series(state, lv)), _rng(_series(state, lv, clear=True))]
        else:
            ser = _series(state, lv, clear=True)
            cells = ["0（屏蔽）" if i2 == masked_axis and max(s) == 0 else _rng(s)
                     for i2, s in enumerate(ser)]
        for cx, c in zip(cols[1:], cells):           # 数值从第 2 列开始，绕开名称
            buf.append(f'<text x="{cx}" y="{yy}" font-size="14.5" fill="#37474f">{esc(c)}</text>')

    edge = ("车体半对角线 0.32 m" if not state.get("cargo")
            else "货框外接半径 0.69 m")
    foot = ("箭头 a→b 为第 0 档到最高档的变化范围；净空 = 边界 − 机器人外缘（%s）；"
            "0 表示边界与货框外缘齐平。" % edge if circ else
            "箭头 a→b 为第 0 档到最高档的变化范围；净空 = 边界 − 机器人外缘"
            "（载货时为货框外缘，非车体）。")
    if masked_axis >= 0:
        foot += "标「屏蔽」的方向边界压在 x = 0，不参与碰撞判定。"
    buf.append(f'<text x="{x}" y="{y + 156}" font-size="13" fill="#78909c">{esc(foot)}</text>')


# ---------------- 单状态大图 ----------------
def render_state(state, path):
    W, H = 1520, 1120
    panel_y, panel_h = 128, 620
    lw, rw = 640, 680
    lx, rx = 50, 790
    multi = len(state["tiers"]) > 1

    xr = max(extent_all(state) * 1.20, 0.8)
    buf = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}" font-family="{FONT}">',
           '<rect width="100%" height="100%" fill="#ffffff"/>', draw_hatch_defs()]

    buf.append(f'<text x="50" y="46" font-size="26" font-weight="700" fill="#1a237e">'
               f'{state["no"]}. {esc(state["name"])}</text>')
    buf.append(f'<text x="50" y="74" font-size="15" fill="#546e7a">'
               f'collision monitor 三级边界　·　分档轴：{esc(state["axis"])}　·　'
               f'车体 0.52×0.36 m</text>')
    draw_legend_h(buf, 60, 96, [(C_L1, 'L1 预警（最外·绿）'), (C_L2, 'L2 减速（橙）'),
                                (C_L3, 'L3 停止（最内·红）')], gap=230)

    # 左：第 0 档三层叠加
    buf.append(f'<rect x="{lx}" y="{panel_y}" width="{lw}" height="{panel_h}" fill="#fafafa" '
               f'stroke="#e0e0e0"/>')
    sub = (f'第 0 档（{state["tiers"][0]}）　三层叠加　·　标注 L1 预警层净空'
           if multi else '三层叠加（无速度分档）　·　标注 L1 预警层净空')
    buf.append(f'<text x="{lx + 16}" y="{panel_y + 28}" font-size="17" font-weight="600" '
               f'fill="#263238">{esc(sub)}</text>')
    v1 = View(lw, panel_h - 20, -xr, xr, -xr * 0.92, xr * 0.92)
    g1 = [f'<g transform="translate({lx},{panel_y + 20})">']
    draw_axes(g1, v1, xr)
    draw_layers(g1, v1, state, 0)
    draw_body_and_cargo(g1, v1, state)
    draw_masked_sector(g1, v1, state, xr, xr * 0.92)
    draw_arrow(g1, v1, state, xr)
    draw_dims(g1, v1, state)
    draw_scalebar(g1, v1, v1.X(-xr * 0.92), v1.Y(-xr * 0.80))
    g1.append('</g>')
    buf += g1

    # 右：各档 L3 叠加
    buf.append(f'<rect x="{rx}" y="{panel_y}" width="{rw}" height="{panel_h}" fill="#fafafa" '
               f'stroke="#e0e0e0"/>')
    sub2 = 'L1 预警层 · 各速度档对比' if multi else 'L1 预警层 · 净空标注'
    buf.append(f'<text x="{rx + 16}" y="{panel_y + 28}" font-size="17" font-weight="600" '
               f'fill="#263238">{esc(sub2)}</text>')
    v2 = View(rw, panel_h - 100, -xr, xr, -xr * 0.92, xr * 0.92)
    g2 = [f'<g transform="translate({rx},{panel_y + 32})">']
    draw_axes(g2, v2, xr)
    if multi:
        draw_tier_outlines(g2, v2, state)
    draw_body_and_cargo(g2, v2, state)
    draw_masked_sector(g2, v2, state, xr, xr * 0.92)
    draw_arrow(g2, v2, state, xr)
    if not multi:
        draw_dims(g2, v2, state)
    g2.append('</g>')
    buf += g2

    if multi:
        draw_legend(buf, rx + 40, panel_y + panel_h - 100,
                    [(TIER_COLORS[t % len(TIER_COLORS)], f'第 {t} 档  {lab}', "8 4")
                     for t, lab in enumerate(state["tiers"])])
    else:
        draw_legend(buf, rx + 40, panel_y + panel_h - 85,
                    [(C_L3, 'L3 停止（最内·红）', None), (C_L2, 'L2 减速（橙）', None),
                     (C_L1, 'L1 预警（最外·绿）', None)])

    # 设计要点 + 三个边界框的距离表
    ny = panel_y + panel_h + 34
    buf.append(f'<text x="50" y="{ny}" font-size="16" font-weight="600" fill="#1a237e">'
               f'设计要点</text>')
    for i, n in enumerate(state["notes"]):
        buf.append(f'<text x="50" y="{ny + 26 + i * 24}" font-size="14.5" fill="#37474f">'
                   f'·　{esc(n)}</text>')
    draw_boundary_table(buf, state, 50, ny + 40 + len(state["notes"]) * 24)
    buf.append('</svg>')

    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(buf))


# ---------------- 九宫格总览 ----------------
def render_overview(path):
    cols, rows = 3, 3
    cw, ch, gap, pad = 500, 380, 16, 40
    W = pad * 2 + cols * cw + (cols - 1) * gap
    H = 92 + pad + rows * ch + (rows - 1) * gap
    buf = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}" font-family="{FONT}">',
           '<rect width="100%" height="100%" fill="#ffffff"/>', draw_hatch_defs()]
    buf.append(f'<text x="{pad}" y="46" font-size="26" font-weight="700" fill="#1a237e">'
               f'轮式机器人 9 种运动状态 · 避障边界总览</text>')
    buf.append(f'<text x="{pad}" y="74" font-size="15" fill="#546e7a">'
               f'各图为第 0 档（或唯一档）三层叠加　·　最外层绿=L1 预警　中间橙=L2 减速　'
               f'最内层红=L3 停止　·　深色=车体　棕色虚线=货框　斜纹=屏蔽区</text>')

    for i, st in enumerate(STATES):
        c, r = i % cols, i // cols
        x, y = pad + c * (cw + gap), 92 + pad + r * (ch + gap)
        buf.append(f'<rect x="{x}" y="{y}" width="{cw}" height="{ch}" fill="#fafafa" '
                   f'stroke="#e0e0e0"/>')
        buf.append(f'<text x="{x + 16}" y="{y + 30}" font-size="18" font-weight="700" '
                   f'fill="#1a237e">{st["no"]}. {esc(st["name"])}</text>')
        sw = extent_all(st) * 1.22
        v = View(cw - 20, ch - 66, -sw, sw, -sw * 0.86, sw * 0.86)
        g = [f'<g transform="translate({x + 10},{y + 50})">']
        draw_layers(g, v, st, 0)
        draw_body_and_cargo(g, v, st)
        draw_masked_sector(g, v, st, sw, sw * 0.86)
        draw_arrow(g, v, st, sw)
        g.append('</g>')
        buf += g
        if st["kind"] == "circle":
            cap = (f"L3 R = {fmt(st['layers'][0][0])} m"
                   f"　净空 {fmt(clearance(st, 0, 0, 2))} m")
        else:
            masked_ax = 0 if st.get("mask") == "front" else (1 if st.get("mask") == "rear" else -1)
            cap = "L3 净空 = 行进向 {} / 反向 {} / 侧向 {} m".format(
                *["0（屏蔽）" if i == masked_ax and st["layers"][0][0][i] == 0.0
                  else fmt(clearance(st, 0, 0, i)) for i in range(3)])
        buf.append(f'<text x="{x + 16}" y="{y + ch - 12}" font-size="13.5" fill="#546e7a">'
                   f'{esc(cap)}</text>')

    buf.append('</svg>')
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(buf))


# ---------------- 参数表 ----------------
def render_spec(path):
    lines = ["| # | 状态 | key | 档位 | 分档轴 | L3 停止 | L2 减速 | L1 预警 |",
             "|---|---|---|---|---|---|---|---|",
             "| | | | | | *下表均为净空（边界 − 机器人外缘），单位 m* | | |"]
    for st in STATES:
        for t, lab in enumerate(st["tiers"]):
            cells = []
            for lv in range(3):                          # 0=L3 1=L2 2=L1
                if st["kind"] == "circle":
                    cells.append(f"R {fmt(st['layers'][t][lv])}"
                                 f"（净空 {fmt(clearance(st, t, lv, 2))}）")
                else:
                    cells.append(" / ".join(
                        fmt(clearance(st, t, lv, i)) for i in range(3)))
            lines.append("| {} | {} | `{}` | {} | {} | {} | {} | {} |".format(
                st["no"], st["name"], st["key"], lab, st["axis"], *cells))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def self_check():
    # 颜色序：索引 0/1/2 必须是 L3/L2/L1，L1 绿且在最外，L3 红且在最内
    assert [n for _lv, n, _c in NAME_OF_LV] == ["L3 停止", "L2 减速", "L1 预警"], NAME_OF_LV
    assert [c for _lv, _n, c in NAME_OF_LV] == [C_L3, C_L2, C_L1]
    assert C_L1 == "#43a047" and C_L3 == "#e53935", f"L1={C_L1} L3={C_L3}"
    assert FILL_OP[0] > FILL_OP[1] > FILL_OP[2], FILL_OP   # 最内层填充最重

    for st in STATES:
        n = st["no"]
        # 1) 三层必须层层包住：同轴单调不减
        for t in range(len(st["tiers"])):
            if st["kind"] == "circle":
                R = [st["layers"][t][lv] for lv in range(3)]
                assert R[0] < R[1] < R[2], f"状态{n} 档{t} 圆半径未递增 {R}"
            else:
                for lv in range(1, 3):
                    for i2, ax in enumerate("前后侧"):
                        lo = st["layers"][t][lv - 1][i2]
                        hi = st["layers"][t][lv][i2]
                        assert hi >= lo - 1e-9, \
                            f"状态{n} 档{t} L{4 - lv} 的{ax}向 {hi} 未包住内层 {lo}"
        # 2) L3 必须含车体；被屏蔽的方向允许为 0
        for t in range(len(st["tiers"])):
            if st["kind"] == "circle":
                assert st["layers"][t][0] >= BODY_R - 1e-9, \
                    f"状态{n} L3 半径 {st['layers'][t][0]} < 车体半对角线 {BODY_R:.3f}"
                continue
            F, B, S = st["layers"][t][0]
            for val, need, ax in ((F, R_F, "前"), (B, R_B, "后"), (S, R_S, "侧")):
                if val == 0.0 and st.get("mask"):
                    continue
                assert val >= need - 1e-9, f"状态{n} 档{t} L3 {ax}向 {val} < 车体 {need}"
        # 3) 屏蔽侧与压平侧一致
        if st.get("mask"):
            F, B, _S = st["layers"][0][0]
            if st["mask"] == "front":
                assert F == 0.0 and B > 0.0, f"状态{n} 屏蔽前方但 F={F} B={B}"
            else:
                assert B == 0.0 and F > 0.0, f"状态{n} 屏蔽后方但 F={F} B={B}"
            xs = [p[0] for p in shape_pts(st, 0, 0)]
            empty = 1 if max(xs) == 0 else (-1 if min(xs) == 0 else 0)
            assert empty != 0, f"状态{n} 未压平到 x=0"
            assert empty == (st["dir"] if st["mask"] == "front" else -st["dir"]), \
                f"状态{n} 斜纹画在了错误的半平面"
        # 4) 载货状态：L3 行进向必须包住货框
        if st.get("cargo") == "travel":
            F = st["layers"][0][0][0]
            need = C_F if st["dir"] > 0 else C_B
            assert F >= need, f"状态{n} L3 行进向 {F} < 货框 {need}"
        # 5) 净空必须非负；屏蔽方向豁免（边界被压到 0，净空为负是预期）
        for t in range(len(st["tiers"])):
            for lv in range(3):
                for i2 in range(3):
                    if st.get("mask") and st["layers"][t][lv][i2] == 0.0:
                        continue
                    c = clearance(st, t, lv, i2)
                    assert c >= -1e-9, \
                        f"状态{n} 档{t} {NAME_OF_LV[lv][1]} 轴向{i2} 净空为负 {c:.3f}"
        # 6) 载货状态的净空量到货框外缘：行进向 L3 可低于车体净空但不得为负
        if st.get("cargo") == "travel":
            assert robot_edge(st, 0) == C_F or robot_edge(st, 1) == C_B, "载货状态方向判据取错"

        # 7) 视图范围必须容得下最外层 L1
        M = extent_all(st)
        for t in range(len(st["tiers"])):
            for lv in range(3):
                if st["kind"] == "circle":
                    vals = (st["layers"][t][lv],) * 3
                else:
                    vals = st["layers"][t][lv]
                assert max(vals) <= M + 1e-9, f"状态{n} 有层超出 L1 外圈"
    print("self_check: ok")


if __name__ == "__main__":
    # 两套尺寸各自自检并生成净空表；SVG 只按仿真档输出（notes 是仿真档文字）。
    for name, spec_name in (("sim", "spec_table.md"),
                            ("real", "spec_table_real.md")):
        apply_profile(name)
        self_check()
        render_spec(os.path.join(OUT, spec_name))
        print(f"wrote {spec_name} ({name})")
    apply_profile("sim")
    for st in STATES:
        p = os.path.join(OUT, f"{st['no']:02d}_{st['key']}.svg")
        render_state(st, p)
        print("wrote", p)
    render_overview(os.path.join(OUT, "00_overview.svg"))
    render_spec(os.path.join(OUT, "spec_table.md"))
    print("wrote", os.path.join(OUT, "00_overview.svg"))
    print("wrote", os.path.join(OUT, "spec_table.md"))
