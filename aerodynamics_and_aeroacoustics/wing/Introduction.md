# Wing Optimization Code – Introduction

## Overview
This repository contains the code developed for
the aerodynamic optimization of the drone's
**wing profiles**.
The main goal is to identify the airfoils that
provide the **lowest possible flight speed** for
a given aircraft weight, while maintaining a
**high aerodynamic efficiency** (lift-to-drag ratio).

---

## Objectives
- Evaluate the aerodynamic performance of multiple airfoil profiles using **XFOIL**.
- Compute the **lift coefficient (CL)** and **drag coefficient (CD)** for each profile.
- For a given target **weight (W)**, determine the flight speed required for equilibrium:
  $`
  L = W = \frac{1}{2} \rho v^2 S C_L
  `$
- Explore different **geometric configurations**, varying:
  - Wing span (b)
  - Mean chord (c)
  such that $`S = b \cdot c`$.

- Identify for each profile the configuration yielding:
  - **Minimum flight speed**
  - **Best aerodynamic efficiency** $` (C_L / C_D) `$

---

## Methodology

1. **Data Source**
   - Input profiles are processed through **XFOIL** to obtain aerodynamic polars (CL, CD vs. angle of attack).

2. **Computation**
   - For each profile:
     - Define a grid of possible span (b) and chord (c) combinations.
     - Compute the corresponding surface $` S = b \cdot c `$.
     - For a given weight $` W `$, compute the required flight speed:
       $`
       v = \sqrt{ \frac{2W}{\rho S C_L} }
       `$
     - Store CL, CD, v, and $` C_L / C_D `$.

3. **Optimization**
   - Filter results to select profiles with:
     - Lowest $` v `$
     - Highest $` C_L / C_D `$
   - Return a ranked list of optimal profiles and configurations.

---

## Expected Output
- A dataset (or table) containing:
  - Airfoil name
  - Span and chord combination
  - $` C_L, C_D, C_L/C_D `$
  - Computed flight speed
- A summary identifying the **best-performing profiles** based on:
  - **Minimum speed**
  - **Maximum aerodynamic efficiency**


