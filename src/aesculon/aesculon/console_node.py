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
        # ASCII Visualization
        bars = int(fb.progress_percent / 5)
        prog_bar = "[" + "=" * bars + " " * (20 - bars) + "]"
        
        # Color codes (ANSI)
        RED = '\033[91m'
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        RESET = '\033[0m'

        t_color = GREEN if abs(fb.current_temp_k - 350) < 50 else RED
        p_color = GREEN if fb.current_pressure_pa < 200000 else RED

        sys.stdout.write(f"\r{prog_bar} {fb.progress_percent:5.1f}% | Temp: {t_color}{fb.current_temp_k:6.1f}K{RESET} | Pres: {p_color}{fb.current_pressure_pa/1000:6.1f} kPa{RESET} | Status: {fb.system_status}")
        
        # Overlay Object Detection Alerts
        if hasattr(fb, 'detected_hazards') and fb.detected_hazards:
            alert_msg = " | ".join(fb.detected_hazards)
            # Clearing line below to print alerts without scrolling mess
            sys.stdout.write(f"\n   [NEURAL OPTICS]: {RED}{alert_msg}{RESET}")
            # Move cursor back up one line 
            sys.stdout.write("\033[F") 
            
        sys.stdout.flush()

    def goal_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            print("\n>>> CRITICAL: PROTOCOL REJECTED BY SAFETY LOGIC.")
            return
        print("\n>>> PROTOCOL UPLOADED. EXECUTING BATCH...")
        self._res_future = self._goal_handle.get_result_async()
        self._res_future.add_done_callback(self.res_cb)

    def res_cb(self, future):
        res = future.result().result
        print(f"\n\n>>> BATCH COMPLETE. CERTIFIED RESULT: {res.verdict_code}")
        rclpy.shutdown()

    def cancel_run(self):
        print("\n>>> MANUAL SCRAM INITIATED (SUB-ROUTINE 0x99).")
        if hasattr(self, '_goal_handle') and self._goal_handle:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            print(">>> SCRAM SIGNAL ACKNOWLEDGED BY CORE.")

def main(args=None):
    rclpy.init(args=args)
    node = AesculonConsole()
    
    # Phase 1: Setup & Protocol Transmission
    try:
        node.send_protocol()
    except KeyboardInterrupt:
        print("\n\n>>> Initialization Aborted.")
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        return

    # Phase 2: Active Monitoring (Safety Loop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    while rclpy.ok():
        try:
            executor.spin()
            if not rclpy.ok(): break
        except KeyboardInterrupt:
            # Clear line and show Safety Menu
            sys.stdout.write("\n")
            print("!!! SAFETY INTERLOCK ENGAGED - SYSTEM PAUSED !!!")
            print("To TRIGGER EMERGENCY SCRAM, type 'CONFIRM'.")
            print("To RESUME normal operation, press Enter.")
            try:
                # Flush input buffer just in case
                sys.stdin.flush()
                user_in = input("Command [CONFIRM/resume]: ")
                if user_in.strip() == 'CONFIRM':
                    node.cancel_run()
                    break
                else:
                    print(">>> RESUMING BATCH...")
                    continue
            except KeyboardInterrupt:
                print("\n>>> FORCED EXIT.")
                node.cancel_run()
                break
    
    # Cleanup
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except:
        pass

if __name__ == '__main__':
    main()
