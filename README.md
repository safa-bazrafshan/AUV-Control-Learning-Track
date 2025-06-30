# Week 1 – LQR Controller for AUV Depth Control

This folder contains the first part of the control learning track for the Autonomous Underwater Vehicle (AUV) system. It focuses on modeling the system using state-space representation and designing a Linear Quadratic Regulator (LQR) to control the depth of the vehicle.

---

## 🔧 Objectives

- Model the AUV depth system using second-order dynamics
- Convert the model into state-space form
- Analyze system stability using eigenvalues
- Design an LQR controller
- Explore the effect of Q and R matrices
- Simulate and evaluate the system's performance

---

## 📂 Files

| File | Description |
|------|-------------|
| auv_state_space_model.m | State-space modeling of AUV depth system and step response |
| lqr_controller.m        | Basic LQR design and simulation |
| lqr_tuning_experiment.m | Compare different Q and R matrices and analyze system behavior |

---

## 📈 Sample Output

Simulated step responses for different Q/R tuning strategies are included in the /images folder.

---

## ▶️ How to Run

1. Open MATLAB
2. Navigate to the Week1_LQR directory
3. Run each .m file and observe the outputs
4. Use saveas(gcf, 'filename.png') to save plots if needed

---

## 📚 Concepts Used

- State-Space Systems
- Eigenvalue Analysis
- LQR (Linear Quadratic Regulator)
- MATLAB Control Toolbox

---

## ✅ Next Step

Move to [Week2_SMC](../Week2_SMC/) to learn about Sliding Mode Control and its robustness to disturbances.
