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
        """
        Interactive prompt to capture thermodynamic parameters from the field engineer.
        """
        print("\n=== AESCULON: SAFETY JUDGMENT SYSTEM (CLI V1.0) ===")
        print(">> ENTER PROTOCOL PARAMETERS FOR SIMULATION <<\n")

        try:
            p_id = input("1. Protocol ID (str) [default: TEST-01]: ") or "TEST-01"
            
            # Input Validation Loop for Temperature
            while True:
                try:
                    t_in = input("2. Target Temperature (Kelvin) [e.g., 350.0]: ")
                    target_k = float(t_in)
                    if target_k < 0:
                        print("   [ERROR] Absolute zero violation. Try again.")
                        continue
                    break
                except ValueError:
                    print("   [ERROR] Invalid input. Please enter a number.")

            # Input Validation Loop for Pressure
            while True:
                try:
                    p_in = input("3. Safety Pressure Limit (Pascals) [e.g., 200000]: ")
                    limit_pa = float(p_in)
                    if limit_pa <= 101325:
                        print("   [WARN] Limit below atmospheric pressure. Are you sure?")
                    break
                except ValueError:
                    print("   [ERROR] Numeric value required.")

            # Input for Duration
            d_in = input("4. Duration (seconds) [default: 30]: ") or "30"
            duration = float(d_in)

        except KeyboardInterrupt:
            print("\n[ABORT] User cancelled input.")
            sys.exit(0)

        # Construct the Goal
        goal = Judgment.Goal()
        goal.protocol_id = p_id
        goal.target_temp_k = target_k
        goal.limit_pressure_pa = limit_pa
        goal.duration_sec = duration

        self.get_logger().info(f'Transmitting Protocol {p_id} to Core...')
        self._client.wait_for_server()
        
        self._future = self._client.send_goal_async(
            goal, 
            feedback_callback=self.telemetry_callback
        )
        self._future.add_done_callback(self.goal_response_callback)

    def telemetry_callback(self, msg):
        fb = msg.feedback
        # Dynamic formatted string for clear engineering readout
        # \r allows overwriting the line for a smooth "dashboard" effect
        sys.stdout.write(
            f"\r>> [{fb.progress_percent:3.0f}%] {fb.system_status} | "
            f"T: {fb.current_temp_k:6.1f} K | "
            f"P: {fb.current_pressure_pa/1000:6.1f} kPa | "
            f"Q_gen: {fb.heat_gen_watts/1000:5.2f} kW   "
        )
        sys.stdout.flush()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("\n\n[ERROR] Protocol Rejected by Core (Check Physics Constraints).")
            sys.exit(1)
        
        print(f"\n[ACK] Core accepted protocol. Simulation running for {goal_handle.request.duration_sec}s...")
        self._res_future = goal_handle.get_result_async()
        self._res_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        res = future.result().result
        print("\n\n" + "="*40)
        print("FINAL JUDGMENT REPORT")
        print("="*40)
        print(f"VERDICT      : {res.verdict_code}")
        print(f"SAFE SHUTDOWN: {res.safe_shutdown}")
        print(f"FINAL YIELD  : {res.final_yield:.2f}%")
        print("="*40 + "\n")
        
        # Shutdown after one run
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AesculonConsole()
    node.send_protocol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # Clean exit
    except Exception as e: # Catch-all for stray ROS errors
        print(f"[SYSTEM FAILURE] {e}")

if __name__ == '__main__':
    main()
