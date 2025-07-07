# 🌊 AUV Depth Control Project – From Classical to Intelligent & Robust Methods

This repository presents a progressive control study of an Autonomous Underwater Vehicle (AUV) depth regulation system. Each phase demonstrates a more advanced control technique, from classical PID to robust and intelligent strategies.

---

## 📌 Control Phases Overview

| Phase | Method                             | Goal |
|-------|-------------------------------------|------|
| 1️⃣   | PID Controller                      | Classical baseline depth control |
| 2️⃣   | Linear Quadratic Regulator (LQR)    | Optimal state-feedback control |
| 3️⃣   | Sliding Mode Control (SMC)          | Robust control against model uncertainties |
| 4️⃣   | Fuzzy Logic + ANFIS                 | Intelligent nonlinear control |
| 5️⃣   | Model Reference Adaptive Control    | Adaptive parameter tuning |
| 6️⃣   | LQR + DOBC                          | Rejection of external disturbances |
| 7️⃣   | Noise Rejection in Sensor Feedback  | Handling sensor noise via filtering |

---

## ⚙️ AUV System Model

The AUV is modeled as a second-order linear system:

\[
m \ddot{z} + b \dot{z} + k z = u(t) + d(t)
\]

Where:
- \( z(t) \): depth  
- \( u(t) \): control input  
- \( d(t) \): external disturbance (optional)  
- \( m, b, k \): mass, damping, hydrostatic stiffness

---

## 🔍 Phase-by-Phase Summary

### ✅ Phase 1 – PID Control

- Basic proportional-integral-derivative control
- Tuned manually: \( K_p = 50, K_i = 5, K_d = 15 \)
- Result: Successful regulation with overshoot

### ✅ Phase 2 – LQR Control

- State-space design minimizing:
  \[
  J = \int (x^T Q x + u^T R u) dt
  \]
- Optimal, smooth convergence

### ✅ Phase 3 – Sliding Mode Control (SMC)

- Defines a sliding surface:
  \[
  s = \dot{e} + \lambda e
  \]
- Robust to parameter uncertainty and external disturbances

### ✅ Phase 4 – Fuzzy Logic & ANFIS

- Manual fuzzy controller: 5×5 rule table
- ANFIS: trained using data from a PID-like law
- Nonlinear, adaptive behavior

### ✅ Phase 5 – MRAC (Adaptive Control)

- Reference model tracking with online parameter updates:
  \[
  \dot{\theta} = -\gamma \phi e
  \]
- Works under unknown model parameters

### ✅ Phase 6 – LQR + DOBC

- Adds a disturbance observer (DO):
  \[
  \hat{d}(t) = \text{LPF}(y - y_{nom})
  \]
- Final control:
  \[
  u = u_{nominal} - \hat{d}(t)
  \]
- Effectively cancels external disturbance

### ✅ Phase 7 – Noise Rejection

- Gaussian sensor noise added to depth feedback
- Filtered using a Moving Average Filter:
  \[
  y_{filtered}(k) = \frac{1}{M} \sum_{i=0}^{M-1} y(k - i)
  \]
- Significantly reduces control chattering due to noise

---

## 📁 Folder Structure

| Folder | Content |
|--------|---------|
| `/pid` | Classical PID controller |
| `/lqr` | Linear Quadratic Regulator |
| `/smc` | Sliding Mode Controller |
| `/fuzzy_vs_anfis_controller` | Fuzzy logic & ANFIS |
| `/mrac` | Adaptive control (MRAC) |
| `/dobc_lqr` | LQR with Disturbance Observer |
| `/noise_rejection` | LQR with sensor noise + filtering |

---

## 🧪 Output Samples

Each phase includes plots:
- Depth tracking
- Control effort
- Parameter estimation (for MRAC)
- Disturbance rejection & estimation
- Noise suppression comparison

---

## 💡 How to Run

1. Open MATLAB  
2. Navigate to the folder of a specific phase  
3. Run the main `.m` file  
4. Review and save results

---

## 👤 Author

**Safa Bazrafshan**  
📧 safa.bazrafshan@gmail.com  
🔗 [ORCID: 0009-0004-4029-9550](https://orcid.org/0009-0004-4029-9550)

---

## 🤝 Contributions

This project is part of an academic and migration-focused learning journey. Feedback, forks, and collaborations are warmly welcome 💙

---