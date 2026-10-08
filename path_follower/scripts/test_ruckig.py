import numpy as np
import matplotlib.pyplot as plt
from ruckig import InputParameter, OutputParameter, Result, Ruckig

# 1) 初始状态 s=0, v=0, a=0；目标状态 s=2, v=1, a=0
dof = 1
inp = InputParameter(dof)
inp.current_position = [0.0]
inp.current_velocity = [1.2]
inp.current_acceleration = [0.7]

inp.target_position = [3.8]
inp.target_velocity = [0.01]
inp.target_acceleration = [0.0]

# 2) 约束：v_max=1, a_max=0.6, jerk_max=1
inp.max_velocity = [0.01]
inp.min_velocity = [0.0]
inp.max_acceleration = [0.8]
# inp.min_acceleration = [-0.6]
inp.max_jerk = [1.0]

# 3) 逐步计算轨迹（周期 0.01s）
dt = 0.01
otg = Ruckig(dof, dt)
out = OutputParameter(dof)

t_list, s_list, v_list, a_list, j_list = [], [], [], [], []

res = Result.Working
while res == Result.Working:
    res = otg.update(inp, out)
    t_list.append(out.time)
    s_list.append(out.new_position[0])
    v_list.append(out.new_velocity[0])
    a_list.append(out.new_acceleration[0])
    j_list.append(out.new_jerk[0])
    out.pass_to_input(inp)

print(f"轨迹总时长: {out.trajectory.duration:.4f}s")

t = np.array(t_list)
s = np.array(s_list)
v = np.array(v_list)
a = np.array(a_list)
j = np.array(j_list)

# 4) 绘图：位置 / 速度 / 加速度 / jerk
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 10))

axes[0].plot(t, v, color="tab:orange")
axes[0].axhline(inp.max_velocity[0], color="gray", linestyle="--", linewidth=1)
axes[0].set_ylabel("velocity v (m/s)")
axes[0].grid(True)

axes[1].plot(t, a, color="tab:green")
axes[1].axhline(inp.max_acceleration[0], color="gray", linestyle="--", linewidth=1)
axes[1].axhline(-inp.max_acceleration[0], color="gray", linestyle="--", linewidth=1)
axes[1].set_ylabel("acceleration a (m/s^2)")
axes[1].grid(True)

axes[2].plot(t, j, color="tab:red")
axes[2].axhline(inp.max_jerk[0], color="gray", linestyle="--", linewidth=1)
axes[2].axhline(-inp.max_jerk[0], color="gray", linestyle="--", linewidth=1)
axes[2].set_ylabel("jerk j (m/s^3)")
axes[2].set_xlabel("time t (s)")
axes[2].grid(True)

fig.suptitle("Ruckig: s=0->2, v=0->1, a=0->0 (v_max=1, a_max=0.6, jerk_max=1)")
fig.tight_layout()
# plt.savefig("test_ruckig.png", dpi=150)
plt.show()
