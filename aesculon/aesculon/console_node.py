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
        goal.target_temp_k = 340.0      # 67 C
        goal.limit_pressure_pa = 200000.0 # 2 Bar approx

        self.get_logger().info('Transmitting Thermodynamic Constraints...')
        self._client.wait_for_server()
        
        self._future = self._client.send_goal_async(
            goal, feedback_callback=self.monitor_physics)
        self._future.add_done_callback(self.goal_ack)

    def monitor_physics(self, msg):
        fb = msg.feedback
        # Engineering Notation for Clarity
        log = (
            f"STATUS: {fb.aesculon_status} | "
            f"T: {fb.temp_k:.1f} K | "
            f"P: {fb.pressure_pa/1000:.1f} kPa | "
            f"Q_gen: {fb.heat_gen_rate/1000:.1f} kW"
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
        self.get_logger().info(f'Conversion: {res.final_conversion:.2f}%')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AesculonConsole()
    node.begin_restoration()
    rclpy.spin(node)
