import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tutorial_interfaces.action import Mission
import asyncio

class FleetManagementClient(Node):
    """
    Simulates a Fleet Manager console that dispatches missions to AMRs.
    Supports asynchronous feedback monitoring and result gathering.
    """

    def __init__(self):
        super().__init__('fleet_management_client')
        self._action_client = ActionClient(self, Mission, 'mission')
        self._goals_completed = 0

    def dispatch_mission(self, mission_type, zone, priority):
        """
        Creates and sends a mission goal to the available AMR server.
        """
        goal_msg = Mission.Goal()
        goal_msg.mission_type = mission_type
        goal_msg.target_zone = zone
        goal_msg.priority = priority
        goal_msg.parameters = "{'speed_limit': 1.5, 'payload_sensitive': true}"

        self.get_logger().info(f'Dispatcher: Waiting for AMR System Uplink...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Dispatcher: AMR System Offline. Aborting dispatch.')
            return

        self.get_logger().info(f'Dispatcher: Transmitting Mission Profile -> [{mission_type}] Zone: {zone}')

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback when server accepts/rejects the goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Dispatcher: Mission Rejected by AMR Controller. Check resource availability.')
            return

        self.get_logger().info('Dispatcher: Mission Accepted. Monitoring Telemetry...')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Process real-time telemetry from the robot.
        """
        data = feedback_msg.feedback
        # Create a "Dashboard" style log
        log = (f"\n--- TELEMETRY STREAM ---\n"
               f"Phase:    {data.current_phase}\n"
               f"Battery:  {data.battery_level:.1f}%\n"
               f"Progress: [{self._progress_bar(data.progress_percentage)}] {data.progress_percentage:.1f}%\n"
               f"System:   {data.log_message}\n"
               f"ETA:      {data.estimated_time_remaining}s\n"
               f"------------------------")
        self.get_logger().info(log)

    def get_result_callback(self, future):
        """
        Process the final mission report.
        """
        result = future.result().result
        status = "SUCCESS" if result.success else "FAILURE"
        
        self.get_logger().info(
            f"\n=== MISSION REPORT ===\n"
            f"Status: {status}\n"
            f"Detail: {result.report}\n"
            f"Efficiency: {result.operational_efficiency}\n"
            f"Energy Used: {result.energy_consumed:.2f}%\n"
            f"Execution Time: {result.execution_time:.2f}s\n"
            f"======================"
        )
        self._goals_completed += 1
        # In a real app, we might signal a shutdown or next task here.
        # For demonstration, we simply shutdown after one mission.
        rclpy.shutdown()

    def _progress_bar(self, percentage, length=20):
        fill = int(length * percentage / 100)
        bar = '█' * fill + '-' * (length - fill)
        return bar

def main(args=None):
    rclpy.init(args=args)

    client = FleetManagementClient()
    
    # We can accept command line args for mission type to make it interactive?
    # For now, hardcode a complex scenario
    
    # Scenario: High Priority Logistics Delivery
    client.dispatch_mission("LOGISTICS_DELIVERY", "Zone-B12", 10)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass # Clean exit
    finally:
        client.destroy_node()

if __name__ == '__main__':
    main()
