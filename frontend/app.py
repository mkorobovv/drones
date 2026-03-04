import numpy as np
import streamlit as st
import psycopg2
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ==========================================
# 1. КОНФИГУРАЦИЯ И ПАРАМЕТРЫ (Constants)
# ==========================================
class Config:
    T_MAX = 5.6
    N_STEPS = 15
    DT = T_MAX / N_STEPS
    X_TARGET = np.array([5, 5, 10, 0, 0, 0])
    ALPHA1 = 0.9 # Вес за цилиндры
    ALPHA2 = 1.6 # Вес за "окно"
    ALPHA3 = 0.9 # Вес за терминальное состояние
    
    # Ограничения (Цилиндры и Окно)
    CYLINDERS = [(1.5, 2.5, 2.5), (6.5, 7.5, 2.5)]
    WINDOW = (4, 5, 0.5)
    
    # Границы управления
    U_LIMITS = [
        (-0.2618, 0.2618), # u1
        (-np.pi, np.pi),   # u2
        (-0.2618, 0.2618), # u3
        (0, 12)            # u4
    ]

class CostCalculator:

    @staticmethod
    def heaviside(a):
        return 1.0 if a > 0 else 0.0



    @staticmethod
    def get_running_cost(x):
        """Интегральная часть f0(t, x, u)"""

        # 1. Штраф за цилиндры (phi_i) [cite: 110]

        penalty_cyl = 0

        for (cx, cz, r) in Config.CYLINDERS:
            phi = r - np.sqrt((cx - x[0])**2 + (cz - x[2])**2)
            penalty_cyl += CostCalculator.heaviside(phi)


        # 2. Штраф за "окно" (eta) [cite: 116]

        wx, wz, wr = Config.WINDOW

        # В модели используется расстояние до центра окна

        dist_to_window = np.sqrt((wx - x[0])**2 + (5 - x[1])**2 + (wz - x[2])**2)

        eta = wr - dist_to_window

        penalty_win = CostCalculator.heaviside(eta)


        return Config.ALPHA1 * penalty_cyl + Config.ALPHA2 * penalty_win



    @staticmethod
    def get_terminal_cost(x_final, tf):
        """Терминальная часть F(tf, x(tf))"""
        dist_sq = np.sum((x_final - Config.X_TARGET)**2)

        return tf + Config.ALPHA3 * dist_sq



# ==========================================
# 2. МОДУЛЬ РАБОТЫ С ДАННЫМИ (Data Layer)
# ==========================================
class DataProvider:
    def __init__(self):
        self.conn_params = {
            "dbname": "trajectory",
            "user": "postgres",
            "password": "postgres",
            "host": "localhost"
        }

    def _execute_query(self, query, params=None):
        with psycopg2.connect(**self.conn_params) as conn:
            with conn.cursor() as cur:
                cur.execute(query, params)
                return cur.fetchall()

    def get_training_data(self, position_id, limit=295):
        query = """
            SELECT ts.state, ts.control 
            FROM trajectory_states ts
            JOIN scores s ON ts.trajectory_id = s.trajectory_id
            WHERE ts.position_id = %s
            ORDER BY s.score ASC LIMIT %s
        """
        data = self._execute_query(query, (position_id, limit))
        if not data: return None, None
        return np.array([r[0] for r in data]), np.array([r[1] for r in data])
    
    def get_best_program_control(self):
        """Получение последовательности управлений лучшей траектории (эталона)"""
        query = """
            SELECT ts.control 
            FROM trajectory_states ts
            JOIN scores s ON ts.trajectory_id = s.trajectory_id
            WHERE s.score = (SELECT MIN(score) FROM scores)
            ORDER BY ts.position_id ASC
        """
        data = self._execute_query(query)
        return np.array([r[0] for r in data])

    def get_initial_states(self, limit=6):
        query = """
            SELECT ts.state FROM trajectory_states ts
            JOIN scores s ON ts.trajectory_id = s.trajectory_id
            WHERE ts.position_id = 1
            ORDER BY s.score ASC LIMIT %s
        """
        return [np.array(row[0]) for row in self._execute_query(query, (limit,))]
    

# ==========================================
# 3. МАТЕМАТИЧЕСКИЙ ЯДРО (Physics & Logic)
# ==========================================
class FlightDynamics:
    @staticmethod
    def ode_system(t, x, u):
        phi, theta, psi, thrust = u
        dx, dy, dz = x[3], x[4], x[5]
        
        dvx = (np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi)) * thrust
        dvy = (np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)) * thrust
        dvz = thrust * np.cos(theta) * np.cos(phi) - 9.805
        
        return [dx, dy, dz, dvx, dvy, dvz]
    
class BundleController:
    def __init__(self, m_features=6):
        self.m = m_features
        self.models = {} # Хранилище коэффициентов для каждого шага

    def _build_basis(self, X_batch):
        """Построение матрицы G (квадратичный базис)"""
        S = X_batch.shape[0]
        
        if self.m == 2:
            x = X_batch[:, [0, 2]]
        else:
            x = X_batch[:, :self.m]
        
        # Генерация парных произведений x_i * x_j
        quad = []
        for i in range(self.m):
            for j in range(i, self.m):
                col = x[:, i] * x[:, j]
                if i == j: col *= 0.5
                quad.append(col)
        
        # Сборка: [Квадраты/Кроссы | Линейные | Константа]
        G = np.column_stack(quad + [x, np.ones(S)])
        return G

    def train(self, data_provider):
        for step in range(Config.N_STEPS):
            X, U = data_provider.get_training_data(step + 1)
            if X is not None:
                G = self._build_basis(X)
                # Решение МНК: R = (G^T G)^-1 G^T U
                self.models[step] = np.linalg.pinv(G) @ U

    def get_control(self, x_current, step):
        if step not in self.models: return np.zeros(4)
        
        # Формирование вектора базиса для текущего состояния
        if self.m == 2:
            x = x_current[[0, 2]]
        else:
            x = x_current[:self.m]
        
        basis = []
        for i in range(self.m):
            for j in range(i, self.m):
                basis.append(0.5 * x[i]**2 if i == j else x[i] * x[j])
        basis.extend(x)
        basis.append(1.0)
        
        u_raw = np.array(basis) @ self.models[step]
        
        # Применение ограничений
        return np.array([
            np.clip(u_raw[i], Config.U_LIMITS[i][0], Config.U_LIMITS[i][1]) 
            for i in range(4)
        ])
    
    def get_basis_labels(self):
        labels = []
        
        coord_names = [f'x{i+1}' for i in range(self.m)]
        if self.m == 2:
            coord_names = ['x1', 'x3']
            
        # 1. Квадратичные и перекрестные члены
        for i in range(self.m):
            for j in range(i, self.m):
                if i == j:
                    labels.append(f"0.5 * {coord_names[i]}²")
                else:
                    labels.append(f"{coord_names[i]} * {coord_names[j]}")
        # 2. Линейные члены
        for i in range(self.m):
            labels.append(coord_names[i])
        # 3. Константа
        labels.append("1 (const)")
        return labels
    
# ==========================================
# 4. ДВИЖОК СИМУЛЯЦИИ (Engine)
# ==========================================
class SimulationEngine:
    def __init__(self, controller):
        self.controller = controller

    def run(self, x0):
        current_x = np.array(x0)
        history_x = [current_x.copy()]
        history_u = []

        for step in range(Config.N_STEPS):
            u = self.controller.get_control(current_x, step)
            sol = solve_ivp(
                FlightDynamics.ode_system, 
                [0, Config.DT], 
                current_x, 
                args=(u,), 
                method='RK45'
            )
            current_x = sol.y[:, -1]
            history_x.append(current_x.copy())
            history_u.append(u)
            
        return np.array(history_x), np.array(history_u)
    
# ==========================================
# 5. ВИЗУАЛИЗАЦИЯ (Reporting)
# ==========================================
class Visualizer:
    @staticmethod
    def plot_bundle(trajectories, initial_points):
        plt.figure(figsize=(10, 7))
        
        # Отрисовка ограничений
        for (cx, cz, r) in Config.CYLINDERS:
            circle = plt.Circle((cx, cz), r, color='orange', alpha=0.3, label='Препятствие')
            plt.gca().add_patch(circle)
        
        wx, wz, wr = Config.WINDOW
        window_circ = plt.Circle((wx, wz), wr, color='black', fill=False, lw=2, label='Окно')
        plt.gca().add_patch(window_circ)

        # Траектории
        for i, traj in enumerate(trajectories):
            plt.plot(traj[:, 0], traj[:, 2], color='royalblue', alpha=0.5, lw=1)
        
        plt.scatter(Config.X_TARGET[0], Config.X_TARGET[2], marker='X', color='red', s=100, label='Цель')
        plt.xlabel('x1'); plt.ylabel('x3'); plt.title('Устойчивость пучка синтезированных траекторий')
        plt.grid(True, ls=':')
        plt.axis('equal')
        plt.show()

# ==========================================
# 3. STREAMLIT UI И ЛОГИКА
# ==========================================

st.set_page_config(page_title="Синтез управления БПЛА", layout="wide")
st.title("СИНТЕЗ УПРАВЛЕНИЯ ПУЧКОМ ТРАЕКТОРИЙ")

st.info("**Справка:** Траектория строится в реальном времени. Изменяйте параметры x₀ в боковом меню для проверки устойчивости синтезированного управления.")

# Кэшируем обучение контроллера
@st.cache_resource
def get_trained_controller(m):
    db = DataProvider() 
    
    ctrl = BundleController(m_features=m)
    
    ctrl.train(db) 
    
    return ctrl

# --- Боковая панель (Sidebar) ---
st.sidebar.header("Параметры моделирования")
m_val = st.sidebar.select_slider("Размерность вектора состояния (m)", options=[1,2, 3, 4, 5, 6], value=6)
controller = get_trained_controller(m_val)

st.sidebar.subheader("Начальное состояние (x₀)")
x1 = st.sidebar.slider("x1", -1.0, 1.0, 0.15)
x2 = st.sidebar.slider("x2", -1.0, 1.0, 0.01)
x3 = st.sidebar.slider("x3", 0.0, 0.2, 0.01)
x4 = st.sidebar.slider("x4", 0.0, 0.2, 0.01)
x5 = st.sidebar.slider("x5", 0.0, 0.2, 0.01)
x6 = st.sidebar.slider("x6", 0.0, 0.2, 0.01)

x0_user = [x1, x2, x3, x4, x5, x6]

# --- Запуск симуляции ---
current_x = np.array(x0_user)
history_x = [current_x.copy()]
history_u = []

for step in range(Config.N_STEPS):
    u = controller.get_control(current_x, step)
    sol = solve_ivp(FlightDynamics.ode_system, [0, Config.DT], current_x, args=(u,), method='RK45')
    current_x = sol.y[:, -1]
    history_x.append(current_x.copy())
    history_u.append(u)

history_x = np.array(history_x)
history_u = np.array(history_u)

# --- Графики ---
tab1, tab2 = st.tabs(["Траектория квадрокоптера", "Изменение управления U"])

with tab1:
    fig_traj, ax = plt.subplots(figsize=(5, 5))
    # Ограничения
    for i, (cx, cz, r) in enumerate(Config.CYLINDERS):
        circle = plt.Circle((cx, cz), r, color='orange', alpha=0.2, label=f'Препятствие {i+1}')
        ax.add_patch(circle)
    wx, wz, wr = Config.WINDOW
    ax.add_patch(plt.Circle((wx, wz), wr, color='black', fill=False, lw=2, ls='--', label='Окно'))
    
    # Траектория
    ax.plot(history_x[:, 0], history_x[:, 2], 'b-o', markersize=4, label='Траектория квадрокоптера')
    ax.scatter(x0_user[0], x0_user[2], marker='o', color='green', s=100, label='Начальное состояние')
    ax.scatter(Config.X_TARGET[0], Config.X_TARGET[2], marker='o', color='red', s=100, label='Целевое состояние')
    
    ax.set_xlabel("x1")
    ax.set_ylabel("x3")
    ax.legend(loc='upper left', prop={'size': 7})
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    st.pyplot(fig_traj)

with tab2:
    fig_u, axs = plt.subplots(2, 2, figsize=(7, 6))
    u_labels = ['u1 (Крен)', 'u2 (Тангаж)', 'u3 (Рыскание)', 'u4 (Тяга)']
    t_u = np.linspace(0, Config.T_MAX, Config.N_STEPS)
    
    for i in range(4):
        curr_ax = axs[i//2, i%2]
        curr_ax.step(t_u, history_u[:, i], where='post', color='green')
        curr_ax.set_title(u_labels[i])
        curr_ax.grid(True, alpha=0.3)
    plt.tight_layout()
    st.pyplot(fig_u)

st.sidebar.divider()
st.sidebar.subheader("Симуляция пучка")
run_bundle = st.sidebar.button("Сгенерировать пучок")

if run_bundle:
    st.divider()
    st.header(f"📊 Анализ устойчивости пучка")
    
    with st.spinner("Запуск симуляции для всего пучка..."):
        # 1. Получаем N начальных состояний из БД
        db = DataProvider()
        bundle_initial_states = db.get_initial_states(limit=6)
        
        # 2. Запускаем симуляцию для каждой точки
        bundle_results = []
        individual_costs = [] # Список для I(x0, d)
        for start_x in bundle_initial_states:
            current_x = np.array(start_x)
            traj = [current_x.copy()]
            integral_sum = 0
            
            for step in range(Config.N_STEPS):
                u = controller.get_control(current_x, step)
                integral_sum += CostCalculator.get_running_cost(current_x) * Config.DT
                sol = solve_ivp(FlightDynamics.ode_system, [0, Config.DT], current_x, args=(u,), method='RK45')
                current_x = sol.y[:, -1]
                traj.append(current_x.copy())
            
            total_i = integral_sum + CostCalculator.get_terminal_cost(current_x, Config.T_MAX)
            individual_costs.append(total_i)
            bundle_results.append(np.array(traj))

        j_avg = np.mean(individual_costs)
        j_max = np.max(individual_costs)

        # Вывод метрик в Streamlit

        col1, col2 = st.columns(2)

        col1.metric("Средний функционал (J_avg)", f"{j_avg:.4f}")

        col2.metric("Гарантирующий функционал (J_max)", f"{j_max:.4f}")

        # 3. Визуализация пучка
        fig_bundle, ax_b = plt.subplots(figsize=(10, 5))
        
        # Отрисовка ограничений
        for i, (cx, cz, r) in enumerate(Config.CYLINDERS):
            ax_b.add_patch(plt.Circle((cx, cz), r, color='orange', alpha=0.15))
        wx, wz, wr = Config.WINDOW
        ax_b.add_patch(plt.Circle((wx, wz), wr, color='black', fill=False, lw=1.5, ls='--'))

        # Отрисовка каждой траектории из пучка
        for i, traj in enumerate(bundle_results):
            label = "Траектории пучка" if i == 0 else None
            ax_b.plot(traj[:, 0], traj[:, 2], color='royalblue', alpha=0.3, linewidth=0.8, label=label)
            # Точка старта для каждой
            ax_b.scatter(traj[0, 0], traj[0, 2], color='green', s=10, alpha=0.5)

        # Цель
        ax_b.scatter(Config.X_TARGET[0], Config.X_TARGET[2], marker='o', color='red', s=100, label='Терминальное состояние (xf)', zorder=5)

        ax_b.set_xlabel("x1")
        ax_b.set_ylabel("x3")
        ax_b.legend(loc='upper left', prop={'size': 8})
        ax_b.grid(True, linestyle=':', alpha=0.4)
        ax_b.set_aspect('equal')
        
        st.pyplot(fig_bundle)
        

st.divider()
st.header("Матрица коэффициентов $r^{(i)}(j)$")

with st.expander("Посмотреть веса модели для каждого шага"):
    # Выбор шага управления
    step_to_view = st.slider("Выберите шаг управления (position_id)", 1, Config.N_STEPS, 1)
    
    # Извлекаем матрицу для выбранного шага
    # step-1 так как в словаре ключи от 0 до 14
    if (step_to_view - 1) in controller.models:
        matrix_R = controller.models[step_to_view - 1]
        
        # Подготовка данных для DataFrame
        row_labels = controller.get_basis_labels()
        col_labels = ['u1 (Крен)', 'u2 (Тангаж)', 'u3 (Рыскание)', 'u4 (Тяга)']
        
        import pandas as pd
        df_R = pd.DataFrame(matrix_R, index=row_labels, columns=col_labels)
        
        # Визуализация с цветовой шкалой (Heatmap)
        st.write(f"Коэффициенты для интервала времени: { (step_to_view-1)*Config.DT:.2f}с - {step_to_view*Config.DT:.2f}с")
        
        # Применяем цветовое оформление: красное — отрицательные, синее — положительные
        st.dataframe(df_R.style.background_gradient(cmap='RdBu', axis=None).format("{:.4f}"), 
                     height=500, use_container_width=True)
        
        st.caption("Подсказка: Интенсивность цвета показывает степень влияния компонента состояния на формирование управления.")
    else:
        st.warning("Сначала обучите модель или проверьте подключение к БД.")

st.sidebar.divider()
compare_btn = st.sidebar.button("Сравнительный анализ")

if compare_btn:    
    db = DataProvider()
    bundle_initial_states = db.get_initial_states(limit=8)
    best_u_program = db.get_best_program_control()
    
    col_synth, col_prog = st.columns(2)
    
    # 1. СИМУЛЯЦИЯ ПУЧКА (СИНТЕЗ - ОБРАТНАЯ СВЯЗЬ)
    results_synth = []
    for start_x in bundle_initial_states:
        curr_x = np.array(start_x)
        traj = [curr_x.copy()]
        for step in range(Config.N_STEPS):
            u = controller.get_control(curr_x, step)
            sol = solve_ivp(FlightDynamics.ode_system, [0, Config.DT], curr_x, args=(u,), method='RK45')
            curr_x = sol.y[:, -1]
            traj.append(curr_x.copy())
        results_synth.append(np.array(traj))

    # 2. СИМУЛЯЦИЯ ПУЧКА (ПРОГРАММНОЕ УПРАВЛЕНИЕ - БЕЗ ОС)
    results_prog = []
    for start_x in bundle_initial_states:
        curr_x = np.array(start_x)
        traj = [curr_x.copy()]
        for step in range(Config.N_STEPS):
            # Применяем управление эталона, игнорируя текущее отклонение x
            u = best_u_program[step] if step < len(best_u_program) else best_u_program[-1]
            sol = solve_ivp(FlightDynamics.ode_system, [0, Config.DT], curr_x, args=(u,), method='RK45')
            curr_x = sol.y[:, -1]
            traj.append(curr_x.copy())
        results_prog.append(np.array(traj))

    # --- ВИЗУАЛИЗАЦИЯ ---
    
    def plot_comparison(results, title, color):
        fig, ax = plt.subplots(figsize=(6, 6))
        for start_x in bundle_initial_states:
            ax.scatter(start_x[0], start_x[2], marker='o', color='green', s=10, alpha=0.5)
        # Отрисовка препятствий
        for cx, cz, r in Config.CYLINDERS:
            ax.add_patch(plt.Circle((cx, cz), r, color='orange', alpha=0.15))
        ax.add_patch(plt.Circle(Config.WINDOW[:2], Config.WINDOW[2], color='black', fill=False, ls='--'))
        
        # Траектории
        for traj in results:
            ax.plot(traj[:, 0], traj[:, 2], color=color, alpha=0.4, lw=1)
            ax.scatter(traj[-1, 0], traj[-1, 2], color='red', s=10) # Конечные точки
            
        ax.scatter(Config.X_TARGET[0], Config.X_TARGET[2], marker='o', color='red', s=100)
        ax.set_title(title)
        ax.set_xlabel("x1"); ax.set_ylabel("x3")
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        return fig

    with col_synth:
        st.pyplot(plot_comparison(results_synth, "Синтезированное управление", "royalblue"))

    with col_prog:
        st.pyplot(plot_comparison(results_prog, "Программное управление", "salmon"))
