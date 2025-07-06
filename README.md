# 🌊 AUV Depth Control Project

This repository contains a step-by-step development of various control strategies for an Autonomous Underwater Vehicle (AUV) to regulate its vertical depth.

The project progresses from classical to robust and intelligent controllers, and compares their performance.

---

## 🎯 Objective

Design, simulate, and evaluate different controllers for underwater depth regulation using MATLAB.

---

## 🧪 Control Phases Implemented

| Phase | Method | Description |
|-------|--------|-------------|
| [PID Controller](./pid_controller/) | Classical | Simple baseline controller using proportional-integral-derivative logic |
| [LQR Controller](./lqr/) | Optimal Control | Minimizes a cost function for optimal tracking and effort |
| [SMC Controller](./smc/) | Robust Control | Strong against disturbances using sliding surface |
| [Fuzzy vs ANFIS](./fuzzy_vs_anfis_controller/) | Intelligent Control | Comparison between manual fuzzy rules and neuro-fuzzy learning (ANFIS) |

---

## 📂 Folder Structure

Each folder contains:

- .m simulation files  
- Output plots  
- Individual README.md with method details

---

## 📈 Sample Results

- PID reaches 5m with small overshoot  
- LQR offers smooth optimal tracking  
- SMC shows fast convergence & disturbance rejection  
- ANFIS shows learning-based fuzzy control, but with some steady-state error

---

## 📚 Next Steps

- Implement adaptive control (MRAC)  
- Explore hybrid and learning-based reinforcement controllers  
- Prepare full scientific paper based on these results

---

## 👤 Author

Safa Bazrafshan  
📧 safa.bazrafshan@gmail.com  
🔗 [ORCID: 0009-0004-4029-9550](https://orcid.org/0009-0004-4029-9550)

---

## 🧠 Keywords

MATLAB · Control Systems · AUV · SMC · LQR · ANFIS · Fuzzy Logic · PID · Intelligent Control
