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
R_GAS = 8.314        # J/(mol*K) - Ideal Gas Constant
E_ACT = 75000.0      # J/mol - Activation Energy (High barrier)
DH_RXN = -2.5e5      # J/mol - Enthalpy of Reaction (Exothermic)
ARRHENIUS_A = 1.0e8  # 1/s - Pre-exponential Factor
VOL_REACTOR = 5.0    # m^3 - Reactor Volume
RHO_MIX = 950.0      # kg/m^3 - Density
CP_MIX = 4200.0      # J/(kg*K) - Specific Heat Capacity
UA_COOL = 8500.0     # W/K - Heat Transfer Coefficient * Area

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
        self.get_logger().info('>>> AESCULON PHYSICS ENGINE ONLINE. THERMODYNAMICS ACTIVE.')

    def goal_callback(self, goal):
        # Physical Constraint: Absolute Zero check
        if goal.target_temperature_kelvin <= 0:
            return GoalResponse.REJECT
        self.get_logger().info(f'Analyzing Protocol: {goal.protocol_id}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('>>> EXTERNAL ENTROPY DETECTED (MANUAL STOP).')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('>>> SYSTEM ISOLATED. INTEGRATING DIFFERENTIAL EQUATIONS.')
        
        # Initial State (Standard Ambient)
        T = 298.15      # K (25 C)
        C_A = 2000.0    # mol/m^3 (Initial Concentration)
        C_A0 = 2000.0   # Reference Initial Concentration
        P_initial = 101325.0 # Pa (1 atm)
        
        # Extract Goal Parameters
        target_T = goal_handle.request.target_temperature_kelvin
        limit_P = goal_handle.request.max_pressure_pascals
        
        dt = 0.5        # Time step (seconds)
        time_elapsed = 0.0
        
        feedback = Judgment.Feedback()
        result = Judgment.Result()

        # SIMULATION LOOP (Run for 60 virtual seconds)
        while time_elapsed < 60.0:
            # 1. CHECK CANCELLATION
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.thermodynamic_state = "INTERRUPTED"
                return result

            # 2. PHYSICS ENGINE (The Heart of Aesculon)
            
            # A. Calculate Reaction Rate Constant (k) via Arrhenius Eq
            # k = A * exp(-Ea / RT)
            k = ARRHENIUS_A * math.exp(-E_ACT / (R_GAS * T))
            
            # B. Calculate Reaction Rate (r) - First Order assumed
            # r = k * C_A
            rate = k * C_A
            
            # C. Mass Balance: dC_A/dt = -r
            # Explicit Euler Integration
            dCa_dt = -rate
            C_A += dCa_dt * dt
            if C_A < 0: C_A = 0.0 # Mass cannot be negative

            # D. Energy Balance: m*Cp*dT/dt = Q_gen - Q_rem
            # Q_gen = Rate * Volume * (-dH)  (Note: dH is negative for exo)
            Q_gen = rate * VOL_REACTOR * (-DH_RXN)
            
            # Q_rem = UA * (T - T_coolant)
            # Logic: If T > Target, Coolant valve opens (T_coolant = 280K)
            # If T < Target, Jacket is passive (T_coolant = T_ambient)
            T_coolant = 280.0 if T > target_T else 298.0
            Q_rem = UA_COOL * (T - T_coolant)
            
            # dT/dt = (Q_gen - Q_rem) / (rho * V * Cp)
            thermal_mass = RHO_MIX * VOL_REACTOR * CP_MIX
            dT_dt = (Q_gen - Q_rem) / thermal_mass
            
            # Apply Stochastic Noise (Real-world sensor fluctuation)
            noise = random.uniform(-0.05, 0.05) 
            T_new = T + (dT_dt * dt) + noise
            T = T_new

            # E. Equation of State (Pressure)
            # P = P_initial * (T / T_initial) + Vapor Pressure Contribution (Antoine)
            # Modeled roughly after volatile organic solvent vapor pressure curve
            P_vapor_contribution = 1000 * math.exp(0.045 * (T - 298.15))
            P_current = P_initial * (T / 298.15) + P_vapor_contribution

            # 3. JUDGMENT LOGIC (SIL-4 SAFETY)
            if P_current > limit_P:
                self.get_logger().error(f'CRITICAL: Pressure {P_current:.0f} Pa > Limit {limit_P} Pa!')
                self.get_logger().error('CRITICAL: VESSEL INTEGRITY THREAT DETECTED.')
                
                goal_handle.abort()
                result.thermodynamic_state = "CONTAINMENT_BREACH_PREVENTED"
                result.final_conversion_efficiency = (1.0 - (C_A / C_A0))
                return result

            # 4. FEEDBACK PUBLICATION
            feedback.current_temp_k = T
            feedback.current_pressure_pa = P_current
            feedback.heat_generation_watts = Q_gen
            feedback.reactant_concentration_molar = C_A
            feedback.reaction_progress = (1.0 - (C_A / C_A0))
            
            goal_handle.publish_feedback(feedback)
            time_elapsed += dt
            time.sleep(0.1) # Running faster than real-time

        # SUCCESS
        goal_handle.succeed()
        result.thermodynamic_state = "EQUILIBRIUM_REACHED"
        result.final_conversion_efficiency = (1.0 - (C_A / C_A0))
        self.get_logger().info('>>> CYCLE COMPLETE. ENTROPY MINIMIZED.')
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
