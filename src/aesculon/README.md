# AESCULON: Advanced Process Safety & Digital Twin Architecture
**Code Name:** PROJECT-SURETRUST | **Safety Integrity Level:** SIL-4
**Version:** 2.0.0 (Enterprise)

<p align="center">
  <h3>Next-Generation Industrial Autonomy</h3>
</p>

## 1. Executive Summary
AESCULON is a state-of-the-art **Distributed Cyber-Physical System (dCPS)** designed for the autonomous supervision of critical chemical batch reactors. Unlike traditional SCADA systems, AESCULON integrates **First-Principles Physics Modeling**, **Stochastic State Estimation (Kalman Filtering)**, and **Agentic Artificial Intelligence** to achieve reliable operation under extreme conditions.

The system is architected on **ROS 2 (Robot Operating System 2)**, leveraging its Data Distribution Service (DDS) middleware for real-time, deterministic communication.

## 2. Key Pillars of Technology

### A. Neural-Grade Physics Engine (`aesculon_core`)
At the heart of AESCULON lies a high-fidelity digital twin of the reactor.
- **Arrhenius Kinetics**: Simulates reaction rates as a function of temperature ($k = Ae^{-E/RT}$).
- **Thermodynamic Energy Balance**: Solves differential equations for heat generation ($Q_{gen}$) vs. removal ($Q_{cool}$) using explicit Euler integration.
- **Stochastic Modeling**: Injects Gaussian noise to simulate real-world sensor jitter, which is then filtered.

### B. Sensor Fusion & Estimation
- **Kalman Filter**: Implements a recursive two-step process (Predict -> Update) to fuse noisy sensor measurements with the physics model's predictions. This provides a "true state" estimate ($x_{hat}$) that is statistically optimal, eliminating signal noise.

### C. Autonomous Supervisory Control (The "Agent")
- **PID Control**: Adaptive Proportional-Integral-Derivative controller for precise thermal regulation.
- **Weibull Reliability Analysis**: A real-time predictive maintenance module that calculates the cumulative hazard function of reactor components based on stress cycles, predicting 'Time-to-Failure' and alerting operators *before* a breakdown occurs.

### D. Distributed Computer Vision
- **Rust/Corrosion Detection**: A dedicated vision node processes telemetry to visualize the reactor vessel. It employs computer vision algorithms (HSV masking, Contouring) to scan for structural defects (rust), classifying them by criticality.
- **Thermal Mapping**: Generates a real-time thermal gradient map of the reactor core for HMI visualization.

### E. Security & Compliance
- **21 CFR Part 11 Audit Trail**: All critical events, user actions, and alarms are cryptographically hashed and logged to `aesculon_secure_audit.log`, ensuring immutability and regulatory compliance.

## 3. System Architecture

The project follows a **Micro-Services Architecture** over the ROS 2 bus:

| Node | Type | Responsibility |
| :--- | :--- | :--- |
| **`aesculon_core`** | Action Server | Physics simulation, Sensor Fusion, PID Control, Safety Logic, Telemetry Publishing. |
| **`aesculon_console`** | Action Client | Mission Control interface. Sends batch protocols, displays ASCII dashboard. |
| **`aesculon_vision`** | Subscriber | GPU-accelerated visualization, Corrosion detection overlay, HUD rendering. |

### Communication Interfaces
- **Action (`aesculon/action/Judgment`)**: Used for the long-running batch process. Supports feedback (Hz), Goal Cancellation, and Result Verification.
- **Topic (`/aesculon/telemetry`)**: High-frequency JSON payload broadcast for the distributed vision system.

## 4. Competitive Advantage (The "Killer" Features)
AESCULON facilitates a paradigm shift from **Reactive** to **Proactive** safety.

| Feature | Legacy Systems (Siemens/Honeywell) | AESCULON (Next-Gen) |
| :--- | :--- | :--- |
| **Update Rate** | 1-2 Hz (Polling) | **10-100 Hz (DDS Events)** |
| **Logic** | Boolean (Thresholds) | **Probabilistic (Kalman/Bayesian)** |
| **Cost** | High ($50k+ Licensing) | **Open Source (ROS 2)** |
| **Flexibility** | Proprietary Vendor Lock-in | **Modular / Vendor-Agnostic** |
| **Cybersecurity**| Obscurity | **Cryptographic Audit Trail** |

## 5. Build & Deployment

**Prerequisites:** ROS 2 Humble/Iron, Python 3.10+, OpenCV, NumPy.

```bash
# 1. Build the Workstation
colcon build --symlink-install

# 2. Source the Overlay
source install/setup.bash
```

## 5. Operations Manual

To initiate the full distributed stack (Core + Console + Vision):

```bash
ros2 launch aesculon ignite.launch.py
```

### Manual Node Activation
If distributed debugging is required:
```bash
# Terminal 1: The Physics Core
ros2 run aesculon core_node

# Terminal 2: The Vision System
ros2 run aesculon vision_node

# Terminal 3: The Operator Console
ros2 run aesculon console_node
```

## 6. Safety Capabilities (SIL-4)
- **Hard-Real-Time Cutoff**: Independent logic path checks pressure limits against `max_safe_pressure` (250kPa) every 10ms.
- **Double-Confirmation Abort**: Manual SCRAM (Emergency Stop) requires a definitive "CONFIRM" handshake to prevent accidental trips.
- **Predictive Hazards**: The UI alerts the operator if the *probability* of failure exceeds 50%, not just when a failure happens.

## 7. Future Roadmap
- **Hardware Integration**: Binding the `VisionSystem` class to physical LiDAR/RGB cameras.
- **Edge Deployment**: Porting `core_node` to micro-ROS on RTOS-based microcontrollers.
- **Deep Learning**: Replacing the HSV Rust Detector with a YOLOv8 standard model.

---
*Built for the Future of Industrial Automation.*

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
