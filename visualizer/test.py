import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ------------------------ 1. Загрузка данных ---------------------------

bundle_path = Path("results_bundle.json")

with open(bundle_path, "r") as f:
    bundle = json.load(f)

inp = bundle["input"]
results = bundle["results"]

N = inp["NumIntervals"]
T = inp["Time"]
dt = T / N

cylinders = inp["Cylinders"]
windows = inp["Windows"]
terminal_state = np.array(inp.get("TerminalState", [0, 0, 0, 0, 0, 0]), dtype=float)

# фильтр по best_score <= 6
valid_results = [
    r for r in results
    if (r["trajectory"] is not None
        and r["best_controls"] is not None
        and r["best_score"] <= 6.2)
]

S = len(valid_results)
print(f"Используем S={S} реализаций")
if not valid_results:
    raise RuntimeError("Нет реализаций с траекторией и best_controls при best_score <= 6.")

n_x = 6
n_u = 4

# --------------------- 2. Квадратичные признаки -----------------------

def quad_features(x):
    """
    φ(x) = [1/2 x1^2, x1 x2, ..., x_{n-1} x_n, 1/2 x_n^2, x1, ..., x_n, 1]
    """
    x = np.asarray(x, dtype=float)
    n = x.size
    feats = []

    for i in range(n):
        for j in range(i, n):
            if i == j:
                feats.append(0.5 * x[i] * x[j])
            else:
                feats.append(x[i] * x[j])

    feats.extend(list(x))
    feats.append(1.0)

    return np.array(feats, dtype=float)

# --------------------- 2b. Квадратичные признаки по (x, t) ------------

def quad_features_xt(x, t, T_total):
    """
    Расширенные признаки по состоянию и времени:
    φ_xt(x, t) = [φ(x), τ, 1/2 τ^2, τ*x1, τ*x2, ..., τ*x_n]
    где τ = t / T_total (нормированное время в [0,1]).
    """
    x = np.asarray(x, dtype=float)
    phi_x = quad_features(x)

    tau = float(t) / float(T_total)
    feats_time = [tau, 0.5 * tau**2]
    feats_cross = list(tau * x)

    return np.concatenate([phi_x, feats_time, feats_cross], dtype=float)


m = len(quad_features(np.zeros(n_x)))
print(f"Размерность признаков m={m}")

# --------------------- 3. Формируем общую систему ---------------------

G_rows = []
U_rows = []

for r in valid_results:
    traj = r["trajectory"]          # список длины N
    ctrls = r["best_controls"]      # список длины N

    for j in range(N):
        x_j = traj[j]
        u_j = ctrls[j]
        G_rows.append(quad_features(x_j))
        U_rows.append(u_j)

G = np.array(G_rows)    # (S*N) x m
U = np.array(U_rows)    # (S*N) x 4

print(f"G shape = {G.shape}, U shape = {U.shape}")

# -------------------- 4. Квадратичная аппроксимация -------------------

# можно добавить слабую регуляризацию (ridge), чтобы сгладить коэффициенты
lam = 1e-6

GtG = G.T @ G
GtU = G.T @ U

R = np.linalg.solve(GtG + lam * np.eye(m), GtU)  # m x 4
coeffs = R.T                                     # (4, m)

# ------------------ 5. Диапазоны управления (сатурация) ---------------

all_u = np.array([u for r in valid_results for u in r["best_controls"]])
u_min = all_u.min(axis=0)
u_max = all_u.max(axis=0)

print("Диапазоны управления по данным:")
for i in range(n_u):
    print(f"u[{i+1}]: [{u_min[i]:.3f}, {u_max[i]:.3f}]")

# ---------------------- 6. Модель квадрокоптера -----------------------

def quadcopter_model(x, u):
    g = 9.81
    u1, u2, u3, u4 = u
    dxdt = np.array([
        x[3],
        x[4],
        x[5],
        (np.cos(u3) * np.sin(u2) * np.cos(u1) + np.sin(u3) * np.sin(u1)) * u4,
        (np.sin(u3) * np.sin(u2) * np.cos(u1) - np.cos(u3) * np.sin(u1)) * u4,
        u4 * np.cos(u2) * np.cos(u1) - g
    ])
    return dxdt

def approx_control(x):
    """Глобальный квадратичный закон u(x)."""
    phi = quad_features(x)
    u = coeffs @ phi   # (4, m) @ (m,) -> (4,)
    return np.clip(u, u_min, u_max)

def simulate_trajectory_feedback(x0):
    x = np.asarray(x0, dtype=float)
    X = [x.copy()]
    U = []

    for j in range(N):
        u = approx_control(x)
        U.append(u)

        k1 = quadcopter_model(x, u)
        k2 = quadcopter_model(x + dt / 2 * k1, u)
        k3 = quadcopter_model(x + dt / 2 * k2, u)
        k4 = quadcopter_model(x + dt * k3, u)

        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        X.append(x.copy())

    return np.array(X).T, np.array(U).T

# --------------------- 7. Цилиндры и окно (x1–x3) ---------------------

def draw_cylinders_and_window(ax):
    """Draw cylindrical obstacles and window with consistent styling."""
    cyl_colors = ['orange', 'purple', '#2c3e50', '#16a085']
    for idx, cyl in enumerate(cylinders):
        x1, _, x3 = cyl["Coordinates"]
        R = cyl["Radius"]
        circle = plt.Circle(
            (x1, x3),
            R,
            color=cyl_colors[idx % len(cyl_colors)],
            fill=False,
            linewidth=2,
            linestyle='-',
            label=f'Препятствие {idx + 1}'
        )
        ax.add_patch(circle)

    for idx, win in enumerate(windows):
        x1, _, x3 = win["Coordinates"]
        R = win["Radius"]
        circle = plt.Circle(
            (x1, x3),
            R,
            color='black',
            fill=False,
            linestyle='--',
            linewidth=2,
            label=f'Окно {idx + 1}'
        )
        ax.add_patch(circle)

    ax.set_xlabel("$x_1$")
    ax.set_ylabel("$x_3$")
    ax.set_aspect("equal", "box")

# --------------------- 8. Траектории для нач. состояний ---------------

fig, ax = plt.subplots(figsize=(9, 8))
draw_cylinders_and_window(ax)

approx_controls_histories = []

for idx, r in enumerate(valid_results):
    x0 = r["initial_state"]
    X_new, U_new = simulate_trajectory_feedback(x0)
    approx_controls_histories.append((idx + 1, U_new.copy()))

    x1 = X_new[0, :]
    x3 = X_new[2, :]

    ax.plot(
        x1,
        x3,
        marker='o',
        markersize=3,
        linewidth=1.5
    )
    init_label = "Начальные состояния" if idx == 0 else None
    ax.scatter(
        x1[0],
        x3[0],
        color='green',
        s=90,
        edgecolors='black',
        zorder=5,
        label=init_label
    )

ax.scatter(
    5,
    10,
    color='red',
    s=120,
    edgecolors='black',
    zorder=6,
    label='Терминальное состояние'
)

ax.legend(loc="best", fontsize=8)
ax.set_title("Траектории с глобальной квадратичной аппроксимацией управления")
ax.grid(True, linestyle='--', alpha=0.7)
ax.set_axisbelow(True)

plt.tight_layout()
plt.show()

if approx_controls_histories:
    sample_idx, sample_u = approx_controls_histories[0]
    t_u = np.linspace(0, T, sample_u.shape[1])
    colors = ['blue', 'green', 'red', 'purple']
    labels = ['u1', 'u2', 'u3', 'u4']

    plt.figure(figsize=(12, 6))
    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.step(
            t_u,
            sample_u[i, :],
            where='post',
            linestyle='dashed',
            color=colors[i],
            label=f'аппроксимированное {labels[i]}'
        )
        plt.xlabel('t')
        plt.ylabel(labels[i])
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend()

    plt.suptitle(f'Изменение управлений U для init #{sample_idx}')
    plt.tight_layout()
    plt.show()

# --------------------- 8b. Траектории из исходного JSON -----------------

fig2, ax2 = plt.subplots(figsize=(9, 8))
draw_cylinders_and_window(ax2)

for idx, r in enumerate(valid_results):
    traj = np.array(r["trajectory"])  # форма (N, n_x)
    x1 = traj[:, 0]
    x3 = traj[:, 2]

    # линия траектории
    ax2.plot(
        x1,
        x3,
        marker='o',
        markersize=3,
        linewidth=1.5
    )

    # начальная точка
    init_label = "Начальные состояния (из JSON)" if idx == 0 else None
    ax2.scatter(
        x1[0],
        x3[0],
        color='green',
        s=90,
        edgecolors='black',
        zorder=5,
        label=init_label
    )

ax2.scatter(
    terminal_state[0],
    terminal_state[2],
    color='red',
    s=120,
    edgecolors='black',
    zorder=6,
    label='Терминальное состояние'
)

ax2.legend(loc="best", fontsize=8)
ax2.set_title("Траектории из исходного (без аппроксимации управления)")
ax2.grid(True, linestyle='--', alpha=0.7)
ax2.set_axisbelow(True)

plt.tight_layout()
plt.show()

# --------------------- 3b. Формируем систему для u(x, t) --------------

G_xt_rows = []
U_rows = []

for r in valid_results:
    traj = r["trajectory"]          # список длины N
    ctrls = r["best_controls"]      # список длины N

    for j in range(N):
        x_j = traj[j]
        u_j = ctrls[j]

        t_j = j * dt                # физическое время
        G_xt_rows.append(quad_features_xt(x_j, t_j, T))
        U_rows.append(u_j)

G_xt = np.array(G_xt_rows)   # (S*N) x m_xt
U = np.array(U_rows)         # (S*N) x 4

m_xt = G_xt.shape[1]
print(f"G_xt shape = {G_xt.shape}, U shape = {U.shape}, m_xt={m_xt}")

# -------------------- 4b. Квадратичная аппроксимация u(x, t) ----------

lam = 1e-6
GtG_xt = G_xt.T @ G_xt
GtU_xt = G_xt.T @ U

R_xt = np.linalg.solve(GtG_xt + lam * np.eye(m_xt), GtU_xt)  # m_xt x 4
coeffs_xt = R_xt.T                                           # (4, m_xt)

# ---------------- 6b. Управление u(x, t) и симуляция ------------------

def approx_control_xt(x, t):
    """Законы управления u(x, t) с учётом времени."""
    phi_xt = quad_features_xt(x, t, T)
    u = coeffs_xt @ phi_xt   # (4, m_xt) @ (m_xt,) -> (4,)
    return np.clip(u, u_min, u_max)


def simulate_trajectory_feedback_xt(x0):
    x = np.asarray(x0, dtype=float)
    X = [x.copy()]
    U_hist = []

    t = 0.0
    for j in range(N):
        u = approx_control_xt(x, t)
        U_hist.append(u)

        k1 = quadcopter_model(x, u)
        k2 = quadcopter_model(x + dt / 2 * k1, u)
        k3 = quadcopter_model(x + dt / 2 * k2, u)
        k4 = quadcopter_model(x + dt * k3, u)

        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        X.append(x.copy())

        t += dt

    return np.array(X).T, np.array(U_hist).T

# --------------------- 8c. Траектории для u(x, t) ---------------------

fig, ax = plt.subplots(figsize=(9, 8))
draw_cylinders_and_window(ax)

approx_controls_histories_xt = []

for idx, r in enumerate(valid_results):
    x0 = r["initial_state"]
    X_new, U_new = simulate_trajectory_feedback_xt(x0)
    approx_controls_histories_xt.append((idx + 1, U_new.copy()))

    x1 = X_new[0, :]
    x3 = X_new[2, :]

    ax.plot(
        x1,
        x3,
        marker='o',
        markersize=3,
        linewidth=1.5
    )
    init_label = "Начальные состояния (u(x, t))" if idx == 0 else None
    ax.scatter(
        x1[0],
        x3[0],
        color='green',
        s=90,
        edgecolors='black',
        zorder=5,
        label=init_label
    )

ax.scatter(
    terminal_state[0],
    terminal_state[2],
    color='red',
    s=120,
    edgecolors='black',
    zorder=6,
    label='Терминальное состояние'
)

ax.legend(loc="best", fontsize=8)
ax.set_title("Траектории с квадратичной аппроксимацией u(x, t)")
ax.grid(True, linestyle='--', alpha=0.7)
ax.set_axisbelow(True)

plt.tight_layout()
plt.show()
