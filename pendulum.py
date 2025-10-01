import numpy as np
import matplotlib.pyplot as plt

# ----------------------------
# 1. Manual Fuzzy Logic System
# ----------------------------
class FuzzyController:
    def __init__(self):
        # Universe of discourse
        self.angle_universe = np.linspace(-1.5, 1.5, 300)      # radians
        self.angvel_universe = np.linspace(-8, 8, 400)        # rad/s
        self.force_universe = np.linspace(-30, 30, 600)       # Newtons

        
        self.angle_mf = self._create_mf(self.angle_universe, [-1.5, -0.75, -0.1, 0, 0.1, 0.75, 1.5])
        self.angvel_mf = self._create_mf(self.angvel_universe, [-8, -4, -1, 0, 1, 4, 8])
        self.force_mf = self._create_mf(self.force_universe, [-30, -15, -5, 0, 5, 15, 30])

        
        self.labels = ['NL', 'NS', 'Z', 'PS', 'PL']

    def _trimf(self, x, a, b, c):
        """Triangular membership function"""
        return np.maximum(0, np.minimum((x - a) / (b - a), (c - x) / (c - b)))

    def _create_mf(self, universe, points):
        """Create 5 triangular MFs from 7 points: [a0, a1, a2, a3, a4, a5, a6]"""
        p = points
        return {
            'NL': self._trimf(universe, p[0], p[1], p[2]),
            'NS': self._trimf(universe, p[1], p[2], p[3]),
            'Z':  self._trimf(universe, p[2], p[3], p[4]),
            'PS': self._trimf(universe, p[3], p[4], p[5]),
            'PL': self._trimf(universe, p[4], p[5], p[6]),
        }

    def fuzzify(self, value, mf_dict, universe):
        """Return membership degrees for each label"""
        return {label: np.interp(value, universe, mf_dict[label]) for label in self.labels}

    def rule_base(self, angle_deg, angvel_deg):
        """
        Returns a dictionary of output label → activation strength
        Uses max-min inference (Mamdani)
        """
        output_activation = {label: 0.0 for label in self.labels}

        # Helper to get max activation for a rule
        def activate(rule_angle, rule_angvel, output_label):
            strength = min(angle_deg[rule_angle], angvel_deg[rule_angvel])
            if strength > output_activation[output_label]:
                output_activation[output_label] = strength

        activate('NL', 'NL', 'NL')
        activate('NL', 'NS', 'NL')
        activate('NL', 'Z',  'NS')
        activate('NL', 'PS', 'Z')
        activate('NL', 'PL', 'PS')

        activate('NS', 'NL', 'NL')
        activate('NS', 'NS', 'NS')
        activate('NS', 'Z',  'NS')
        activate('NS', 'PS', 'Z')
        activate('NS', 'PL', 'PS')

        activate('Z', 'NL', 'NS')
        activate('Z', 'NS', 'NS')
        activate('Z', 'Z',  'Z')
        activate('Z', 'PS', 'PS')
        activate('Z', 'PL', 'PS')

        activate('PS', 'NL', 'NS')
        activate('PS', 'NS', 'Z')
        activate('PS', 'Z',  'PS')
        activate('PS', 'PS', 'PS')
        activate('PS', 'PL', 'PL')

        activate('PL', 'NL', 'NS')
        activate('PL', 'NS', 'Z')
        activate('PL', 'Z',  'PS')
        activate('PL', 'PS', 'PL')
        activate('PL', 'PL', 'PL')

        return output_activation

    def defuzzify(self, output_activation):
        """Centroid (center of gravity) defuzzification"""
        # aggregated output fuzzy set
        aggregated = np.zeros_like(self.force_universe)
        for i, label in enumerate(self.labels):
            strength = output_activation[label]
            if strength > 0:

                clipped = np.minimum(strength, self.force_mf[label])
                aggregated = np.maximum(aggregated, clipped)

        numerator = np.trapezoid(aggregated * self.force_universe, self.force_universe)
        denominator = np.trapezoid(aggregated, self.force_universe)
        if denominator == 0:
            return 0.0
        return numerator / denominator

    def compute(self, angle, angvel):
        # Fuzzify inputs
        angle_deg = self.fuzzify(angle, self.angle_mf, self.angle_universe)
        angvel_deg = self.fuzzify(angvel, self.angvel_mf, self.angvel_universe)

        # Apply rules
        output_act = self.rule_base(angle_deg, angvel_deg)

        # Defuzzify
        return self.defuzzify(output_act)

# ----------------------------
# 2. Pendulum Physics
# ----------------------------
class InvertedPendulum:
    def __init__(self):
        self.g = 9.81
        self.l = 0.5
        self.m = 0.2
        self.M = 1.0
        self.dt = 0.02

    def dynamics(self, state, force):
        x, x_dot, theta, theta_dot = state
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        den = self.M + self.m * sin_t**2
        theta_ddot = (self.g * sin_t - cos_t * (force + self.m * self.l * theta_dot**2 * sin_t) / den) / \
                     (self.l * (4/3 - (self.m * cos_t**2) / den))
        x_ddot = (force + self.m * self.l * (theta_dot**2 * sin_t - theta_ddot * cos_t)) / den
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

    def step(self, state, force):
        return state + self.dt * self.dynamics(state, force)

# ----------------------------
# 3. Interactive Simulation
# ----------------------------
class InteractivePendulum:
    def __init__(self):
        self.pendulum = InvertedPendulum()
        self.fuzzy = FuzzyController()
        self.state = np.array([0.0, 0.0, 0.05, 0.0])  # [x, x_dot, theta, theta_dot]
        self.dragging = False
        self.target_x = 0.0
        self.paused = False
        self.x_min, self.x_max = -2.0, 2.0

        # Plot (no grid!)
        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(-0.2, 1.2)
        self.ax.set_aspect('equal')
        self.ax.set_title("Interactive Inverted Pendulum (Manual Fuzzy Logic)\nDrag the cart!", fontsize=12)

        self.cart_plot, = self.ax.plot([], [], 'ks', markersize=20)
        self.rod_plot, = self.ax.plot([], [], 'o-', color='tab:blue', linewidth=4, markersize=8)

        # Events
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.timer = self.fig.canvas.new_timer(interval=20)
        self.timer.add_callback(self.update)
        self.timer.start()

    def on_click(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        cart_x = self.state[0]
        if abs(event.xdata - cart_x) < 0.3 and abs(event.ydata) < 0.2:
            self.dragging = True
            self.target_x = np.clip(event.xdata, self.x_min, self.x_max)

    def on_release(self, event):
        self.dragging = False

    def on_motion(self, event):
        if self.dragging and event.inaxes == self.ax:
            self.target_x = np.clip(event.xdata, self.x_min, self.x_max)

    def on_key(self, event):
        if event.key == ' ':
            self.paused = not self.paused
            title = "Interactive Inverted Pendulum (Manual Fuzzy Logic)\nDrag the cart!"
            if self.paused:
                title += " [PAUSED]"
            self.ax.set_title(title, fontsize=12)
            plt.draw()

    def update(self):
        if self.paused:
            return

        x, x_dot, theta, theta_dot = self.state

        fuzzy_force = self.fuzzy.compute(theta, theta_dot)

        drag_force = 0.0
        if self.dragging:
            k_p, k_d = 20.0, 8.0
            error = self.target_x - x
            drag_force = k_p * error - k_d * x_dot

        total_force = fuzzy_force + drag_force

        new_state = self.pendulum.step(self.state, total_force)
        new_x = new_state[0]

        if new_x <= self.x_min:
            new_x = self.x_min
            new_state[0] = new_x
            new_state[1] = max(0.0, new_state[1])
        elif new_x >= self.x_max:
            new_x = self.x_max
            new_state[0] = new_x
            new_state[1] = min(0.0, new_state[1])

        self.state = new_state

        x = self.state[0]
        pend_x = x + self.pendulum.l * np.sin(self.state[2])
        pend_y = self.pendulum.l * np.cos(self.state[2])

        self.cart_plot.set_data([x], [0])
        self.rod_plot.set_data([x, pend_x], [0, pend_y])

        # Auto-reset if fallen
        if abs(self.state[2]) > np.pi / 2:
            self.state = np.array([0.0, 0.0, 0.05, 0.0])

        self.fig.canvas.draw()

    def show(self):
        plt.show()

# ----------------------------
if __name__ == "__main__":
    sim = InteractivePendulum()
    sim.show()