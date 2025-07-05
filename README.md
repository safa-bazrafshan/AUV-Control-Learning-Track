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

## 🚀 Next

Future steps include implementation of intelligent controllers such as:
- Fuzzy logic
- Adaptive fuzzy (ANFIS)
- Model Reference Adaptive Control (MRAC)

All implementations will be added to this repository progressively.


---

Developed by: Safa Bazrafshan  
📧 safa.bazrafshan@gmail.com  
🔗 [ORCID Profile](https://orcid.org/0009-0004-4029-9550)  
🔗 [LinkedIn](https://www.linkedin.com/in/safa-bazrafshan-04100a29a)  
🔗 [GitHub](https://github.com/safa-bazrafshan)
