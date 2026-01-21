# AESCULON: Thermodynamic Integrity Agent
**Project:** SURE TRUST Project 2 (Action-Based Task Executor)  
**Developer:** Sourish Senapati, Chemical Engineering, Jadavpur University

## 1. Scientific Objective
To implement a **First-Principles Deterministic Safety Agent** that governs a batch chemical reactor. Unlike heuristic controllers, AESCULON solves the **Differential Energy Balance** and **Arrhenius Kinetics** equations in real-time to predict and prevent thermal runaway.

## 2. Mathematical Model
The Action Server integrates the following system of equations:

* **Kinetics:** $k = A \cdot e^{\frac{-E_a}{RT}}$
* **Mass Balance:** $\frac{dC_A}{dt} = -k C_A$
* **Energy Balance:** $m C_p \frac{dT}{dt} = \dot{Q}_{gen} - \dot{Q}_{rem}$
    * Where $\dot{Q}_{gen} = Rate \cdot V \cdot (-\Delta H_{rxn})$
    * Where $\dot{Q}_{rem} = UA(T - T_{coolant})$

## 3. ROS 2 Architecture
* **Action Interface:** `Judgment.action` defines thermodynamic setpoints (Kelvin, Pascals).
* **Aesculon Core (Server):** The physics engine. It introduces stochastic sensor noise to simulate real-world instrumentation limits.
* **Console (Client):** The Human-Machine Interface for protocol injection.

## 4. Execution
1.  `colcon build --packages-select aesculon`
2.  `source install/setup.bash`
3.  `ros2 launch aesculon ignite.launch.py`
