# AESCULON: Thermodynamic Process Control System
**Project:** SURE TRUST Project 2 (Action-Based Task Executor)  
**Developer:** Sourish Senapati, Chemical Engineering, Jadavpur University

## 1. Scientific Objective
To implement a **First-Principles Deterministic Controller** for a batch chemical reactor. This project utilizes the **Differential Energy Balance** and **Arrhenius Kinetics** equations to simulate thermal dynamics and enforce Safety Integrity Level (SIL) constraints.

## 2. Mathematical Model
The system analyzes a **Lumped Parameter Model**, assuming homogeneous temperature and concentration distributions. The simulation solves the following Ordinary Differential Equations (ODEs) using an explicit Euler integration scheme:

### Energy Balance Equation
$$ \frac{dT}{dt} = \frac{\dot{Q}_{gen} - \dot{Q}_{rem}}{m \cdot C_p} $$

Where:
*   **Heat Generation ($\dot{Q}_{gen}$):** $Rate \cdot V \cdot (-\Delta H_{rxn})$ (Exothermic contribution)
*   **Heat Removal ($\dot{Q}_{rem}$):** $UA \cdot (T - T_{coolant})$ (Cooling jacket duty)
*   **Accumulation:** Represented by the thermal mass constant $m \cdot C_p$

### Chemical Kinetics
The reaction rate follows the **Arrhenius Law**, creating a non-linear coupling between temperature and reaction speed:
$$ k = A \cdot \exp\left(\frac{-E_a}{R \cdot T}\right) $$

## 3. ROS 2 Architecture
**Rational for Action Interface:**

Chemical reactions are time-variant dynamic processes requiring:
1.  **Continuous State Estimation (Feedback):** Real-time monitoring of $dT/dt$ derivatives to predict thermal runaway.
2.  **Asynchronous Interruptibility (Cancellation):** Immediate abortion of the process during safety violations (Manual Scram).

*   **Action Interface:** `Judgment.action` defines the thermodynamic process parameters.
*   **Aesculon Core (Server):** The physics engine and logic solver.
*   **Console (Client):** The Command Line Interface (CLI) for protocol injection.

## 4. Execution Instructions
The system operates as a client-server simulation.

### Step 1: Initialize Physics Engine and Visualization
In Terminal 1:
```bash
ros2 launch aesculon dashboard.launch.py
```
*Note: This launches the `aesculon_core` node and `rqt_plot` for real-time telemetry.*

### Step 2: Initialize Operator Client
In Terminal 2:
```bash
ros2 run aesculon aesculon_console
```

### Step 3: Simulation Parameters
The CLI requests the following inputs:

**Test Case A (Nominal Operation):**
*   Target Temperature: `340` K
*   Pressure Limit: `300000` Pa
*   Duration: `30` s
*   **Expected Result:** Temperature stabilizes at setpoint. Status: `OPTIMAL`.

**Test Case B (Runaway Condition):**
*   Target Temperature: `500` K
*   Pressure Limit: `150000` Pa
*   Duration: `40` s
*   **Expected Result:** Exponential pressure rise triggers safety trip. Status: `OVER_PRESSURE_SCRAM`.
