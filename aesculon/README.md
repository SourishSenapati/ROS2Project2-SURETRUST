# AESCULON: Thermodynamic Integrity Agent
**Project:** SURE TRUST Project 2 (Action-Based Task Executor)  
**Developer:** Sourish Senapati, Chemical Engineering, Jadavpur University

## 1. Scientific Objective
To implement a **First-Principles Deterministic Safety Agent** that governs a batch chemical reactor. Unlike heuristic controllers, AESCULON solves the **Differential Energy Balance** and **Arrhenius Kinetics** equations in real-time to predict and prevent thermal runaway.

## 2. Mathematical Model
The system utilizes a **Lumped Parameter Model**, assuming uniform spatial temperature and concentration within the reactor vessel. This allows us to use Ordinary Differential Equations (ODEs) rather than Partial Differential Equations.

The core physics engine performs an explicit Euler integration of the **First Laws of Thermodynamics**:

### Energy Balance Equation
$$ \frac{dT}{dt} = \frac{\dot{Q}_{gen} - \dot{Q}_{rem}}{m \cdot C_p} $$

Where:
*   **Heat Generation ($\dot{Q}_{gen}$):** $Rate \cdot V \cdot (-\Delta H_{rxn})$ (Exothermic contribution)
*   **Heat Removal ($\dot{Q}_{rem}$):** $UA \cdot (T - T_{coolant})$ (Cooling jacket duty)
*   **Accumulation:** Represented by the thermal mass constant $m \cdot C_p$

### Chemical Kinetics
The reaction rate follows the **Arrhenius Law**, creating a non-linear coupling between temperature and reaction speed:
$$ k = A \cdot \exp\left(\frac{-E_a}{R \cdot T}\right) $$

## 3. ROS 2 Architecture Defense
**Why use an Action Server instead of a Service?**

Chemical reactions are **time-variant dynamic processes** that evolve over minutes or hours. They require:
1.  **Continuous State Monitoring (Feedback):** The operator must see the temperature rise ($dT/dt$) in real-time to detect runaway conditions before they become critical.
2.  **Interruptibility (Cancellation):** In a "Scram" event, the process must be aborted immediately. Atomic Services are "fire-and-forget" and cannot provide this safety-critical functionality.

*   **Action Interface:** `Judgment.action` defines the thermodynamic envelope.
*   **Aesculon Core (Server):** The physics engine acting as the SIL-4 Safety Controller.
*   **Console (Client):** The Distributed Control System (DCS) interface.

## 4. Execution
1.  `colcon build --packages-select aesculon`
2.  `source install/setup.bash`
3.  `ros2 launch aesculon ignite.launch.py`
