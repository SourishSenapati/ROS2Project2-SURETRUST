"""
AESCULON CORE: First-Principles Thermodynamic Simulation Node.
Implements Arrhenius Kinetics and Energy Conservation Laws.
"""
import time
import math
import random
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from aesculon.action import Judgment

# --- THERMODYNAMIC CONSTANTS ---
R_GAS = 8.314        # J/(mol*K)
E_ACT = 75000.0      # J/mol - Activation Energy
DH_RXN = -2.5e5      # J/mol - Enthalpy (Exothermic)
ARRHENIUS_A = 1.0e8  # 1/s
VOL_REACTOR = 5.0    # m^3
RHO_MIX = 950.0      # kg/m^3
CP_MIX = 4200.0      # J/(kg*K)
UA_COOL = 8500.0     # W/K

class AesculonCore(Node):
    def __init__(self):
        super().__init__('aesculon_core')
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, Judgment, 'judge_process',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )
        self.get_logger().info('>>> AESCULON PHYSICS ENGINE ONLINE. READY FOR INPUT.')

    def goal_callback(self, goal):
        # Sanity check: Reject absolute zero or negative time
        if goal.target_temp_k <= 0 or goal.duration_sec <= 0:
            return GoalResponse.REJECT
        self.get_logger().info(f'Analyzing Protocol: {goal.protocol_id}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('>>> INTERRUPT SIGNAL RECEIVED.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('>>> INTEGRATING DIFFERENTIAL EQUATIONS.')
        
        # Initial State
        T = 298.15      # K
        C_A = 2000.0    # mol/m^3
        P_initial = 101325.0 # Pa
        
        target_T = goal_handle.request.target_temp_k
        max_P = goal_handle.request.limit_pressure_pa
        total_time = goal_handle.request.duration_sec
        
        dt = 0.2        # Simulation time step
        time_elapsed = 0.0
        
        feedback = Judgment.Feedback()
        result = Judgment.Result()

        while time_elapsed < total_time:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.safe_shutdown = False
                result.verdict_code = "MANUAL_ABORT"
                return result

            # --- PHYSICS ENGINE START ---
            k = ARRHENIUS_A * math.exp(-E_ACT / (R_GAS * T))
            rate = k * C_A
            
            # Mass Balance
            dCa_dt = -rate
            C_A += dCa_dt * dt
            if C_A < 0: C_A = 0

            # Energy Balance
            Q_gen = rate * VOL_REACTOR * (-DH_RXN)
            
            # Logic: PID-like cooling (Active if T > Target)
            T_coolant = 280.0 if T > target_T else 298.0
            Q_rem = UA_COOL * (T - T_coolant)
            
            thermal_mass = RHO_MIX * VOL_REACTOR * CP_MIX
            dT_dt = (Q_gen - Q_rem) / thermal_mass
            
            # Sensor Noise (Stochastic Human-like behavior)
            noise = random.uniform(-0.1, 0.1) 
            T += (dT_dt * dt) + noise

            # Pressure State (PV=nRT approx)
            P_vapor = 1000 * math.exp(0.05 * (T - 298.15))
            P_current = P_initial * (T / 298.15) + P_vapor
            # --- PHYSICS ENGINE END ---

            # SAFETY TRIP
            if P_current > max_P:
                self.get_logger().error(f'!!! VIOLATION: {P_current/1000:.1f} kPa > Limit !!!')
                goal_handle.abort()
                result.safe_shutdown = False
                result.verdict_code = "OVER_PRESSURE_SCRAM"
                result.final_yield = (1.0 - (C_A / 2000.0)) * 100.0
                return result

            # FEEDBACK
            feedback.current_temp_k = T
            feedback.current_pressure_pa = P_current
            feedback.heat_gen_watts = Q_gen
            feedback.progress_percent = (time_elapsed / total_time) * 100.0
            
            if abs(T - target_T) < 5.0:
                feedback.system_status = "STABLE"
            elif T < target_T:
                feedback.system_status = "HEATING"
            else:
                feedback.system_status = "COOLING/EXO"
            
            goal_handle.publish_feedback(feedback)
            time_elapsed += dt
            time.sleep(0.1) # Simulate real-time speed

        goal_handle.succeed()
        result.safe_shutdown = True
        result.verdict_code = "OPTIMAL"
        result.final_yield = (1.0 - (C_A / 2000.0)) * 100.0
        return result

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = AesculonCore()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
