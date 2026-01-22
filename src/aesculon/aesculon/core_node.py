#!/usr/bin/env python3
import time
import math
import random
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from aesculon.action import Judgment

# THERMODYNAMIC CONSTANTS
R_GAS = 8.314
E_ACT = 75000.0
DH_RXN = -2.5e5
ARRHENIUS_A = 1.0e8
VOL_REACTOR = 5.0
RHO_MIX = 950.0
CP_MIX = 4200.0
UA_COOL = 8500.0

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
        self.get_logger().info('>>> AESCULON PHYSICS ENGINE ONLINE.')

    def goal_callback(self, goal):
        self.get_logger().info(f'Accepted Protocol: {goal.protocol_id}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('>>> MANUAL INTERRUPT RECEIVED.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('>>> STARTING PHYSICS INTEGRATION.')
        
        # Initial State
        T = 298.15
        C_A = 2000.0
        P_initial = 101325.0
        
        target_T = goal_handle.request.target_temp_k
        max_P = goal_handle.request.limit_pressure_pa
        total_time = goal_handle.request.duration_sec
        
        dt = 0.2
        time_elapsed = 0.0
        
        feedback = Judgment.Feedback()
        result = Judgment.Result()

        while time_elapsed < total_time:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.safe_shutdown = False
                result.verdict_code = "MANUAL_ABORT"
                return result

            # PHYSICS MATH
            k = ARRHENIUS_A * math.exp(-E_ACT / (R_GAS * T))
            rate = k * C_A
            C_A += (-rate * dt)
            if C_A < 0: C_A = 0

            Q_gen = rate * VOL_REACTOR * (-DH_RXN)
            T_coolant = 280.0 if T > target_T else 298.0
            Q_rem = UA_COOL * (T - T_coolant)
            
            thermal_mass = RHO_MIX * VOL_REACTOR * CP_MIX
            dT_dt = (Q_gen - Q_rem) / thermal_mass
            
            noise = random.uniform(-0.1, 0.1) 
            T += (dT_dt * dt) + noise

            P_vapor = 1000 * math.exp(0.05 * (T - 298.15))
            P_current = P_initial * (T / 298.15) + P_vapor

            # SAFETY CHECK
            if P_current > max_P:
                self.get_logger().error(f'!!! PRESSURE VIOLATION: {P_current:.0f} > {max_P:.0f}')
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
            feedback.system_status = "STABLE" if abs(T - target_T) < 5.0 else "TRANSIENT"
            
            goal_handle.publish_feedback(feedback)
            time_elapsed += dt
            time.sleep(0.1)

        goal_handle.succeed()
        result.safe_shutdown = True
        result.verdict_code = "OPTIMAL_COMPLETION"
        result.final_yield = (1.0 - (C_A / 2000.0)) * 100.0
        return result

def main(args=None):
    rclpy.init(args=args)
    node = AesculonCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down AESCULON Physics Engine...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
