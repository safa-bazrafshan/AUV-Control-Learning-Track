# 🌊 AUV Depth Control – From Classical to Adaptive Controllers

This repository contains MATLAB code and simulation results for designing and evaluating five control strategies to regulate the depth of an Autonomous Underwater Vehicle (AUV).

The project progresses from basic PID control to more advanced and intelligent methods such as Fuzzy Logic, ANFIS, and MRAC.

---

## 📌 Control Phases

| Phase | Method                          | Description |
|-------|----------------------------------|-------------|
| 1️⃣   | PID Controller                   | Classical feedback control using proportional, integral, and derivative terms. |
| 2️⃣   | Linear Quadratic Regulator (LQR) | Optimal state-feedback design minimizing a cost function. |
| 3️⃣   | Sliding Mode Control (SMC)       | Nonlinear robust control strategy with disturbance rejection. |
| 4️⃣   | Fuzzy Logic & ANFIS              | Intelligent controllers based on fuzzy rule sets and adaptive neuro-fuzzy learning. |
| 5️⃣   | Model Reference Adaptive Control | Adaptive controller that updates its parameters in real time to follow a desired model. |

---

## 📁 Folder Structure

Each folder contains the MATLAB scripts and output plots relevant to the corresponding phase.

| Folder | Content |
|--------|---------|
| /pid | PID controller implementation and depth tracking plot. |
| /lqr | LQR design using state-space model and performance analysis. |
| /smc | SMC with and without external disturbance, and control effort plots. |
| /fuzzy_vs_anfis_controller | Includes both fuzzy logic rule-based controller and ANFIS-based system. |
| /mrac | MRAC implementation with damping, tracking reference model, and adaptive parameter plots. |

---

## ⚙️ Techniques Summary

### 🔧 PID Controller
- Simple and widely used.
- Provides a baseline for comparison.
- May show overshoot and slower convergence.

### 📐 Linear Quadratic Regulator (LQR)
- Minimizes:  
  \[
  J = \int (x^T Q x + u^T R u) dt
  \]
- Requires accurate state-space model.
- Produces smooth and optimal response.

### 🧱 Sliding Mode Control (SMC)
- High robustness against uncertainties and external disturbances.
- Uses a sliding surface:
  \[
  s = \dot{e} + \lambda e
  \]
- Control law:
  \[
  u = u_{eq} - \eta \cdot \tanh(k s)
  \]
- May cause chattering, handled by smoothing functions.

### 🤖 Fuzzy Logic & ANFIS
- Fuzzy: Rule-based, interpretable, expert-tuned.
- ANFIS: Trained using datasets, combines fuzzy systems with learning algorithms.
- Suitable for nonlinear, unknown, or complex models.

### 🔄 MRAC (Model Reference Adaptive Control)
- Tracks desired behavior from a predefined model.
- Parameters adapt over time based on tracking error.
- Final implementation includes damping and smooth convergence:
  \[
  \dot{\theta} = -\gamma \cdot \phi(t) \cdot e(t)
  \]

---

## 📈 Results Overview

Each phase includes time-domain response plots such as:

- AUV depth tracking (vs reference model)
- Control effort
- Adaptive parameter convergence (for ANFIS and MRAC)

Plots are saved as .png inside each folder.

---

## 📘 How to Use

1. Open MATLAB.
2. Navigate to the desired folder.
3. Run the main script (e.g., pid_controller.m, mrac_depth_control_final.m)
4. Review and export the plots.

---

## 👤 Author

Safa Bazrafshan  
📧 safa.bazrafshan@gmail.com  
🔗 [ORCID: 0009-0004-4029-9550](https://orcid.org/0009-0004-4029-9550)

---

## 💬 License & Contribution

This repository is open for learning and academic collaboration. Feedback and contributions are welcome.
