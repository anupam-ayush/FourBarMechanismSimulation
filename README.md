# Four-Bar Linkage Mechanism Simulator ⚙️

An interactive, browser-based kinematic simulation and analysis tool for planar four-bar mechanisms. Built using pure JavaScript, HTML5 Canvas, and CSS, this simulator performs full **position, velocity, and acceleration analyses** using vector loop-closure equations and numerical solver methods.

---

## 🌐 Live Demo
[Visit the Website](https://anupam-ayush.github.io/FourBarMechanismSimulation/)


## 🔥 Features

- **Kinematic Analysis Engine:** Real-time computation of joint positions, angular velocities ($\omega$), angular accelerations ($\alpha$), and linear joint dynamics ($v, a$).
- **Grashof Condition Classification:** Automatically detects mechanism types:
  - *Crank-Rocker*
  - *Double Crank (Drag-link)*
  - *Double Rocker*
  - *Non-Grashof Triple Rocker*
  - *Parallelogram & Change Point Mechanisms*
- **Mechanism Inversion:** Toggle any of the four links ($l_1, l_2, l_3, l_4$) as the fixed ground frame.
- **Robust Numerical Solver:** Uses **Freudenstein's Equation** with a **Newton-Raphson** root finder, backed by a **Bisection method** fallback for singularity/toggle point handling.
- **Coupler Curve & Trajectory Tracing:** Visually tracks the real-time path traces of moving joints $B$ and $C$.
- **Interactive Canvas Controls:** Mouse wheel zoom ($0.5\times - 2.0\times$) and click-and-drag panning.
- **Auto-Scaling Viewport:** Dynamic spatial normalization ensuring the linkage fits cleanly on screen regardless of dimensions.

---

## 🧮 Mathematical Foundations

### 1. Position Analysis & Loop Closure
The mechanism is modeled using vector loop closure:

$$\vec{r}_2 + \vec{r}_3 = \vec{r}_1 + \vec{r}_4$$

Using **Freudenstein's Equation**, the output link angle $\theta_4$ is solved relative to input crank angle $\theta_2$:

$$k_1 \cos\theta_4 - k_2 \cos\theta_2 + k_3 = \cos(\theta_4 - \theta_2)$$

Where:
$$k_1 = \frac{l_1}{l_2}, \quad k_2 = \frac{l_1}{l_4}, \quad k_3 = \frac{l_1^2 + l_2^2 - l_3^2 + l_4^2}{2 l_2 l_4}$$

Once $\theta_4$ is obtained, coupler angle $\theta_3$ is determined via:

$$\theta_3 = \operatorname{atan2}\left(l_4 \sin\theta_4 - l_2 \sin\theta_2, \; l_1 + l_4 \cos\theta_4 - l_2 \cos\theta_2\right)$$

### 2. Velocity & Acceleration Matrix System
Differentiating loop closure equations yields the velocity matrix system:

$$\begin{bmatrix} -l_3 \sin\theta_3 & l_4 \sin\theta_4 \\ l_3 \cos\theta_3 & -l_4 \cos\theta_4 \end{bmatrix} \begin{bmatrix} \omega_3 \\ \omega_4 \end{bmatrix} = \begin{bmatrix} l_2 \omega_2 \sin\theta_2 \\ -l_2 \omega_2 \cos\theta_2 \end{bmatrix}$$

Similarly, taking the second derivative yields the linear system solved for angular accelerations $\alpha_3$ and $\alpha_4$:

$$\begin{bmatrix} -l_3 \sin\theta_3 & l_4 \sin\theta_4 \\ l_3 \cos\theta_3 & -l_4 \cos\theta_4 \end{bmatrix} \begin{bmatrix} \alpha_3 \\ \alpha_4 \end{bmatrix} = \begin{bmatrix} B_1 \\ B_2 \end{bmatrix}$$

---

## 💻 Tech Stack

- **Frontend:** HTML5, CSS3 (Inter Font, CSS Flexbox)
- **Scripting:** Modern JavaScript (ES6+)
- **Linear Algebra Library:** [Math.js](https://mathjs.org/) (used for matrix inversion and vector unit conversion)
- **Rendering Engine:** HTML5 Canvas API

---

## 📂 Project Structure

```
FourBarMechanismSimulation/
├── index.html   # Main application markup & UI layout
├── script.js    # Kinematic equations, solvers, canvas rendering logic
└── style.css    # Responsive UI styling and dashboard layout
```

---

## 🚀 How to Run

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/anupam-ayush/FourBarMechanismSimulation.git](https://github.com/anupam-ayush/FourBarMechanismSimulation.git)
   ```
2. **Open the project:**
   Simply double-click `index.html` or open it in any web browser. No local web server or build steps required!

---

## 🎮 How to Use

1. Enter desired lengths for Ground ($l_1$), Crank ($l_2$), Coupler ($l_3$), and Rocker ($l_4$).
2. Select the fixed link (Inversion) from the dropdown.
3. Set angular velocity ($\omega_2$) and step size ($\Delta\theta_2$).
4. Click **Start Simulation** to run the motion loop.
5. Use mouse scroll to **zoom** and drag to **pan** around the canvas workspace.
6. Observe live kinematic telemetry in the telemetry panel.
