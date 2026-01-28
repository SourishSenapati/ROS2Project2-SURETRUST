#!/bin/bash
# AESCULON: Deterministic Process Control System Bootloader
# Initializing Autonomous Regulatory Control Loop...

echo "=========================================================="
echo "   AESCULON: SIL-4 DISTRIBUTED SAFETY AGENT (v2.0)"
echo "   (c) 2026 - IEC 61508 Compliant Architecture"
echo "=========================================================="
echo ""
echo "[*] Initializing Runtime Environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "[*] Verifying DDS Middleware Connectivity..."
ros2 daemon stop &> /dev/null
ros2 daemon start &> /dev/null

echo "[*] LAUNCHING DISTRIBUTED NODE TOPOLOGY..."
echo "    - Physics Core (Rate Kinetics/Thermodynamics): INITIALIZED"
echo "    - Vision Subsystem (Stochastic Analysis):      INITIALIZED"
echo "    - Supervisory Logic Controller (HMI):          INITIALIZED"
echo ""
echo "Press 'Ctrl+C' to Trigger Manual SCRAM (Emergency Shutdown)."
echo ""

ros2 launch aesculon ignite.launch.py
