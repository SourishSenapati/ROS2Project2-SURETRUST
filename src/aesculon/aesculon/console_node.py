#!/usr/bin/env python3
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from aesculon.action import Judgment

class AesculonConsole(Node):
    def __init__(self):
        super().__init__('aesculon_console')
        self._client = ActionClient(self, Judgment, 'judge_process')

    def send_protocol(self):
        print("\n=== AESCULON: SAFETY JUDGMENT SYSTEM ===")
        try:
            p_id = input("Protocol ID: ") or "TEST-01"
            t_in = input("Target Temp (K) [e.g. 350]: ")
            p_in = input("Max Pressure (Pa) [e.g. 200000]: ")
            d_in = input("Duration (sec) [e.g. 30]: ")
            
            goal = Judgment.Goal()
            goal.protocol_id = p_id
            goal.target_temp_k = float(t_in)
            goal.limit_pressure_pa = float(p_in)
            goal.duration_sec = float(d_in)
        except:
            print("Invalid Input.")
            return

        self.get_logger().info('Sending Protocol...')
        self._client.wait_for_server()
        self._future = self._client.send_goal_async(goal, feedback_callback=self.fb_cb)
        self._future.add_done_callback(self.goal_cb)

    def fb_cb(self, msg):
        fb = msg.feedback
        sys.stdout.write(f"\r>> [{fb.progress_percent:.0f}%] T:{fb.current_temp_k:.1f}K | P:{fb.current_pressure_pa:.0f}Pa")
        sys.stdout.flush()

    def goal_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            print("\nGoal Rejected.")
            return
        print("\nGoal Accepted.")
        self._res_future = self._goal_handle.get_result_async()
        self._res_future.add_done_callback(self.res_cb)

    def res_cb(self, future):
        res = future.result().result
        print(f"\n\nFINAL VERDICT: {res.verdict_code}")
        rclpy.shutdown()

    def cancel_run(self):
        print("\n>>> MANUAL ABORT INITIATED.")
        if hasattr(self, '_goal_handle') and self._goal_handle:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            print(">>> ABORT SIGNAL SENT.")

def main(args=None):
    rclpy.init(args=args)
    node = AesculonConsole()
    try:
        node.send_protocol()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
