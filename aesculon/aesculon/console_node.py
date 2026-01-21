import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from aesculon.action import Judgment

class AesculonConsole(Node):
    def __init__(self):
        super().__init__('aesculon_console')
        self._client = ActionClient(self, Judgment, 'judge_process')

    def begin_restoration(self):
        goal = Judgment.Goal()
        goal.protocol_id = "THERMO-CYCLE-ALPHA"
        goal.target_temperature_kelvin = 340.0      # 67 C
        goal.max_pressure_pascals = 200000.0 # 2 Bar approx

        self.get_logger().info('Transmitting Thermodynamic Constraints...')
        self._client.wait_for_server()
        
        self._future = self._client.send_goal_async(
            goal, feedback_callback=self.monitor_physics)
        self._future.add_done_callback(self.goal_ack)

    def monitor_physics(self, msg):
        fb = msg.feedback
        # DCS Style Logging
        log = (
            f"TELEMETRY: T={fb.current_temp_k:.1f}K | "
            f"P={fb.current_pressure_pa/1000:.1f}kPa | "
            f"Q_gen={fb.heat_generation_watts/1000:.1f}kW"
        )
        self.get_logger().info(log)

    def goal_ack(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Aesculon Rejected Constraints.')
            return
        self.get_logger().info('Constraints Accepted. Physics Engine Engaged.')
        self._res_future = goal_handle.get_result_async()
        self._res_future.add_done_callback(self.final_report)

    def final_report(self, future):
        res = future.result().result
        self.get_logger().info('--- THERMODYNAMIC REPORT ---')
        self.get_logger().info(f'State: {res.thermodynamic_state}')
        self.get_logger().info(f'Conversion Efficiency: {res.final_conversion_efficiency:.4f}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AesculonConsole()
    node.begin_restoration()
    rclpy.spin(node)
