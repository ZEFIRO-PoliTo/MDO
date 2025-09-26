# 🛩️ MDO Project – Fuselage & Volume Optimization

## 📖 Overview
This project aims to:
1. **Generate fuselages parametrically** using OpenVSP and Python scripting.
2. **Place 10 volumes inside a fuselage** based on input data.
3. **Check collisions** between fuselage and volumes and apply minimum-distance constraints.
4. **Optimize fuselage volume** to the smallest possible while containing all volumes.
5. **Optimize with additional constraints** (some volumes cannot be close to others).
6. **Compute drag coefficient (Cd)** for each valid configuration with OpenVSP and select the best solutions.

At the end, we will have a **pipeline** that takes:
- Fuselage parameters (H, W, L)
- 10 defined volumes

The system returns the **best configurations** based on minimum fuselage volume and drag coefficient.

---

## 📆 Milestones & Deadlines

| Milestone | Deadline | Deliverables |
|----------|----------|-------------|
| **Milestone 1** | **1 Oct** | Task 1 (fuselage iterator) + Task 2 (volume distribution) |
| **Milestone 2** | **6 Oct** | Task 3 (collision detection + constraints) |
| **Milestone 3** | **15 Oct** | Task 4 (volume minimization) |
| **Milestone 4** | **20 Oct** | Task 5 (constraints) + Task 6 (Cd analysis + full pipeline) |

> **⚠️ Important:** push code frequently so I can review and test well before each deadline.

---

## 🛠️ Setup

### 1. Install Python
- Required version: **Python 3.11**
- [Download here](https://www.python.org/downloads/release/python-3110/)

### 2. Install OpenVSP
- Download from [https://openvsp.org/download.html](https://openvsp.org/download.html)
- Follow installation instructions for your OS
- Make sure you can open OpenVSP manually before scripting.

### 3. Install OpenVSP Python API
Check OpenVSP documentation:
- [Python API Guide](https://openvsp.org/wiki/doku.php?id=python)

