# Classical and Robust Control for AUV Depth Regulation

This folder contains the MATLAB implementation and simulation results for designing two control strategies to regulate the depth of an Autonomous Underwater Vehicle (AUV):

1. Linear Quadratic Regulator (LQR)
2. Sliding Mode Control (SMC)

Each method is implemented step-by-step and includes analysis of performance, robustness, and control effort.

---

## ⚙️ System Model

The AUV depth dynamics are modeled as a second-order linear system:

m * z''(t) + b * z'(t) + k * z(t) = u(t)

Where:
- m: mass of the vehicle  
- b: damping coefficient  
- k: hydrostatic stiffness  
- u(t): control input (thrust)

---

## 📐 LQR Controller

The LQR controller is designed using a state-space representation of the system. It minimizes a cost function to provide an optimal trade-off between state error and control effort.

Key Steps:
- Define A, B, Q, R matrices
- Solve Riccati equation
- Compute gain matrix K
- Simulate step response

File:
- lqr_controller.m

---

## 🧱 Sliding Mode Control (SMC)

Sliding Mode Control offers robust performance against model uncertainties and external disturbances. A sliding surface is defined, and the control law ensures convergence to and motion along this surface.

Sliding surface:

s(t) = de(t) + λ * e(t)

Control law:

u(t) = u_eq - η * tanh(k * s(t))

Key Features:
- High robustness  
- Low sensitivity to parameter variations  
- Chattering reduction via tanh

Files:
- smc_basic_stable.m: SMC with equivalent control  
- smc_with_disturbance.m: SMC under sinusoidal disturbance  
- smc_control_plot.m: Visualization of control effort

---

## 📈 Output Plots

| Plot | Description |
|------|-------------|
| smc_depth_response.png | Depth tracking under SMC with disturbance |
| smc_control_signal.png | Control input behavior in SMC |

> Plots show fast convergence to reference depth (5 m), with small overshoot and strong disturbance rejection.

---

## 💡 Summary

| Method | Performance | Robustness | Complexity |
|--------|-------------|------------|------------|
| LQR | Smooth, optimal | Moderate | Low |
| SMC | Fast, aggressive | High | Medium |

---


# AUV Depth Control – Fuzzy Rule-Based Controller 

This folder contains the implementation of a fuzzy rule-based controller for the vertical depth control of an Autonomous Underwater Vehicle (AUV).

### ✅ Goal
Design and simulate a Mamdani-style fuzzy controller using expert-defined rules, without training or learning (non-ANFIS). The controller aims to stabilize the AUV at a target depth (5 meters).

---

### 🧠 Method

- Inputs:  
  - e → Depth error  
  - de → Change in error (approximated via AUV velocity)

- Output:  
  - u → Control force applied to the AUV

- System parameters:  
  - Mass = 5 kg  
  - Damping = 4 Ns/m  
  - Stiffness = 3 N/m  

- 5 triangular membership functions per input:  
  NL, NS, ZE, PS, PL

- 5×5 fuzzy rule table manually designed

- A scalar gain of 2.5 applied to the defuzzified output to enhance the control strength

---

### 📈 Results

The controller successfully stabilizes the AUV at approximately 3.8–4 meters with low overshoot and smooth convergence.

> Although the target depth (5 m) is not perfectly reached, the system response remains stable and acceptable, making the controller usable as a baseline approach.

<p align="center">
  <img src="Fuzzy_Controller_Depth_Response.png" width="600"/>
</p>

---

### 📁 Files

- fuzzy_rule_based_controller.m → Main simulation code  
- Fuzzy_Controller_Depth_Response.png → Simulation plot

---

### 🔗 Previous Phases
- [Phase 1: PID Controller](../lqr/README.md)
- [Phase 2: LQR Controller](../smc/README.md)

---

### 👤 Author

Safa Bazrafshan  
safa.bazrafshan@gmail.com  
[ORCID: 0009-0004-4029-9550](https://orcid.org/0009-0004-4029-9550)

---
