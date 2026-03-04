import json
import numpy as np
import matplotlib.pyplot as plt

with open("result.json", "r", encoding="utf-8") as f:
    data = json.load(f)

initial_state = np.array(data["initial_state"])        # [6]
trajectory = np.array(data["trajectory"])              # (N+1, 6)
best_controls = np.array(data["best_controls"])        # (N, 4)
best_score = data["best_score"]

N = best_controls.shape[0]         # число интервалов управления
T = 5.6                            # общее время (как в твоем примере)

t_u = np.linspace(0, T, N)

plt.figure(figsize=(12, 6))
colors = ['blue', 'green', 'red', 'purple']
labels = ['u1', 'u2', 'u3', 'u4']

for i in range(4):
    plt.subplot(2, 2, i + 1)
    plt.step(
        t_u,
        best_controls[:, i],
        where='post',
        label=f'кусочно-постоянное {labels[i]}',
        linestyle='dashed'
    )
    plt.xlabel('t')
    plt.ylabel(labels[i])
    plt.legend()
    plt.grid(True)

plt.suptitle(f'Изменение управлений U (best_score = {best_score:.4f})')
plt.tight_layout()
plt.show()

X = trajectory

x0 = initial_state
x_target = np.array([5, 5, 10, 0, 0, 0])
cylinders = [(1.5, 2.5, 2.5), (6.5, 7.5, 2.5)]
window = (4, 5, 0.1)

def plot_circle(center, radius, color, label):
    circle = plt.Circle(center, radius, color=color, fill=False,
                        linestyle='--', label=label)
    plt.gca().add_patch(circle)

plt.figure(figsize=(9, 9))

plt.plot(X[:, 0], X[:, 2],
         label='Траектория',
         marker='o', markersize=4)

plt.scatter(x0[0], x0[2], color='green',
            label='Начальное состояние x0', s=120,
            edgecolors='black', zorder=5)
plt.scatter(x_target[0], x_target[2], color='red',
            label='Терминальное состояние xf', s=120,
            edgecolors='black', zorder=5)

plot_circle((window[0], window[1]), window[2], 'black', 'Окно')
plot_circle((cylinders[0][0], cylinders[0][1]), cylinders[0][2], 'orange', 'Препятствие 1')
plot_circle((cylinders[1][0], cylinders[1][1]), cylinders[1][2], 'purple', 'Препятствие 2')

plt.xlabel('x1')
plt.ylabel('x3')
plt.title('Траектория квадрокоптера')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.7)
plt.axis('equal')
plt.show()
