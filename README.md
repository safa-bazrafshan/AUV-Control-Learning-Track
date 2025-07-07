# 🌊 AUV Depth Control – From Classical to Adaptive and Robust Strategies

This repository contains the MATLAB implementation and simulation results for six control strategies applied to depth regulation of an Autonomous Underwater Vehicle (AUV).

Each phase introduces a more advanced or robust method — moving from classical control to intelligent and adaptive designs.

---

## 📌 Control Phases Overview

| Phase | Method                          | Purpose |
|-------|----------------------------------|---------|
| 1️⃣   | PID Controller                   | Classical baseline for depth control |
| 2️⃣   | Linear Quadratic Regulator (LQR) | Optimal state-feedback design |
| 3️⃣   | Sliding Mode Control (SMC)       | Robust control against modeling uncertainties |
| 4️⃣   | Fuzzy Logic & ANFIS              | Intelligent nonlinear control |
| 5️⃣   | Model Reference Adaptive Control | Adaptive real-time parameter adjustment |
| 6️⃣   | DOBC (Disturbance Observer + LQR)| Robust control against unknown external disturbances |

---

## ⚙️ System Description

The AUV is modeled as a second-order system:

\[
\ddot{z}(t) = \frac{1}{m}(u(t) - b \dot{z}(t) - k z(t)) + d(t)
\]

Where:
- \( z(t) \): Depth of the AUV
- \( u(t) \): Control input (thrust)
- \( d(t) \): External disturbance (e.g., water current)

---

## 🔍 Phase Summaries

### ✅ Phase 1: PID Controller

- Implements classical PID law:
  \[
  u(t) = K_p e + K_i \int e\,dt + K_d \dot{e}
  \]
- Provides a baseline with reasonable tracking and moderate overshoot

### ✅ Phase 2: Linear Quadratic Regulator (LQR)

- State-space controller minimizing:
  \[
  J = \int (x^T Q x + u^T R u)\,dt
  \]
- Produces smooth and optimal convergence

### ✅ Phase 3: Sliding Mode Control (SMC)

- Uses a sliding surface:
  \[
  s = \dot{e} + \lambda e
  \]
- Robust against parameter variations and disturbances

### ✅ Phase 4: Fuzzy Logic & ANFIS

- Fuzzy: Manual rule-based with triangular MFs
- ANFIS: Trained using PID-like control data and genfis3
- Nonlinear control with human-like reasoning

### ✅ Phase 5: MRAC (Model Reference Adaptive Control)

- Online parameter adaptation:
  \[
  \dot{\theta} = -\gamma \phi e
  \]
- Tracks a reference model even with parameter mismatch

### ✅ Phase 6: DOBC with LQR

- Observes external disturbance:
  \[
  \hat{d}(t) = \text{LPF}(y - y_{nom})
  \]
- Applies:
  \[
  u(t) = u_{nominal} - \hat{d}(t)
  \]
- Improves performance in presence of unknown disturbances

---

## 📁 Folder Structure

Each phase is in a dedicated folder:

| Folder | Content |
|--------|---------|
| `/pid` | PID controller |
| `/lqr` | LQR control |
| `/smc` | Sliding Mode Control |
| `/fuzzy_vs_anfis_controller` | Fuzzy + ANFIS methods |
| `/mrac` | Adaptive control (MRAC) |
| `/dobc_lqr` | DOBC + LQR implementation |

---

## 📊 Sample Plots

Each folder contains `.png` files showing:
- Depth tracking
- Control effort
- Disturbance estimation or parameter convergence

---

## 📘 How to Use

1. Open MATLAB
2. Navigate to the folder for the desired phase
3. Run the `.m` files
4. Observe and export results

---

## 👤 Author

**Safa Bazrafshan**  
📧 safa.bazrafshan@gmail.com  
🔗 [ORCID: 0009-0004-4029-9550](https://orcid.org/0009-0004-4029-9550)

---

## 💡 Contribution

This project is open for educational use and scientific discussion. Feel free to explore, fork, or suggest improvements.

---