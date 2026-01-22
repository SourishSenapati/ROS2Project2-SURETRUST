# AESCULON: Chemical Reactor Safety System (SIL-4)

## 1. Project Title and Objective
**Project:** AESCULON (Advanced Embedded System for Chemical Uncontrolled Load Operational Neutralization)
**Objective:** To implement a deterministic Safety Integrity Level (SIL) agent for industrial batch reactors using **ROS 2 Actions**. The system monitors thermodynamic parameters (Temperature, Pressure) in real-time, integrating Arrhenius kinetics ($k = Ae^{-E/RT}$) to predict runaway reactions and triggers an automatic SCRAM (Safety Control Rod Axe Man) shutdown if critical pressure limits are violated.

## 2. System Architecture (ROS Graph)

The system consists of two primary nodes utilizing a client-server architecture via the `aesculon/action/Judgment` interface.

### **Node 1: `aesculon_core` (Action Server)**
- **Role:** The Physics Engine & Safety Logic Controller.
- **Function:**
    - Simulates the batch reactor physics using explicit Euler integration (dt=0.1s).
    - Solves Energy Balance: $dT/dt = (Q_{gen} - Q_{removed}) / (m C_p)$.
    - Calculates Vapor Pressure using the Antoine Equation approximation.
    - **Safety Logic:** If $P_{current} > P_{limit}$, it immediately aborts the goal and scrams the reactor.
- **Interface:** Hosting the `judge_process` action server.

### **Node 2: `aesculon_console` (Action Client)**
- **Role:** Human Machine Interface (HMI) / Operator Console.
- **Function:**
    - Allows the chemical engineer to submit a "Process Protocol" (Goal).
    - Displays live telemetry (Feedback) in a DCS-style format.
    - Receives the final safety verdict (Result).
- **Interface:** Sends goals to `judge_process`.

## 3. Usage of ROS 2 Concepts

| Concept | Usage in AESCULON | Justification (Why not Service?) |
| :--- | :--- | :--- |
| **Action** | The entire batch process (`measure -> integrate -> check safety`) is encapsulated in the `judge_process` action. | Chemical reactions satisfy the "Long Running Task" criteria (>30 mins). A Service would block the main thread, freezing the HMI and preventing the operator from seeing the temperature rise until it was too late. |
| **Goal** | `protocol_id`, `target_temp_k`, `limit_pressure_pa`, `duration_sec` | Represents the startup parameters for a specific chemical batch. |
| **Feedback** | `current_temp_k`, `current_pressure_pa`, `progress_percent` | allows the operator to monitor the reaction curve ($dT/dt$) in real-time to detect runaway trends. |
| **Result** | `safe_shutdown` (bool), `verdict_code` (string), `final_yield` (float) | Provides the final quality assurance certification and safety log for the batch. |
| **Cancellation** | Operator passes `Ctrl+C` handling to sending a Cancel Request. | Essential for manual intervention. If a seal leaks, the operator must abort immediately. A Service cannot be cancelled once started. |

## 4. Build and Run Instructions

### **Build**
```bash
# From workspace root
colcon build --symlink-install
source install/setup.bash
```

### **Run**
**Option 1: Launch Everything (One Command)**
```bash
ros2 launch aesculon dashboard.launch.py
```

**Option 2: Manual Operations (Two Terminals)**
*Terminal 1 (Physics Engine):*
```bash
ros2 run aesculon aesculon_core
```

*Terminal 2 (Operator Console):*
```bash
ros2 run aesculon aesculon_console
```

### **Demonstration Steps**
1. **Start Core**: Ensure the physics engine is online.
2. **Start Console**: Run the client.
3. **Input Protocol**:
    - ID: `BATCH-RX-2026`
    - Target Temp: `350` (Kelvin)
    - Max Pressure: `200000` (Pa)
    - Duration: `30` (Seconds)
4. **Observe**: Watch the pressure rise as the reaction exotherms.
5. **Test Safety**: Run a second batch with a low Max Pressure (e.g. `102000` Pa) to trigger an automatic SCRAM.
