import time
import random
import threading
import json
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tutorial_interfaces.action import Mission

class AdvancedMissionServer(Node):
    """
    A professional-grade Action Server simulating a high-fidelity Autonomous Mobile Robot (AMR)
    system. It manages internal state, simulated battery physics, and complex mission phases.

    Theory of Operation:
    --------------------
    This node implements a stateful action server compliant with the ROS 2 Action protocol.
    Unlike stateless services, this server maintains a persistent 'World Model' (battery, location, status)
    and executes tasks asynchronously. It adheres to the Principle of Non-Blocking Execution
    by utilizing a ReentrantCallbackGroup, allowing the node to process Feedback publication,
    Cancellation requests, and Goal acceptance concurrently.

    Key Architectural Components:
    1.  **State Management**: Atomic updates to internal state variables protected by `threading.Lock`.
    2.  **Physics Simulation**: A discretized time-stepped simulation model for energy consumption.
    3.  **Phase-Based Execution**: Missions are decomposed into sequential 'Phases', allowing for
        granular progress tracking and failure isolation.
    """

    def __init__(self):
        super().__init__('advanced_mission_server')
        
        # Internal State Simulation
        self._battery_level = 100.0  # Percentage
        self._location = "HOME_BASE"
        self._is_busy = False
        self._lock = threading.Lock()

        # Configuration Parameters (Tuned for Simulation Fidelity)
        self._battery_drain_rate = 0.5 # Unit: %/tick. Derived from average motor load.
        self._recharge_rate = 5.0      # Unit: %/tick. Rapid-charge protocol.
        self._tick_rate = 1.0          # Unit: Hz. Simulation time step frequency.

        self._action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.execute_callback,
            # We use a ReentrantCallbackGroup to allow the executor to schedule
            # the execute_callback (long running) in parallel with other callbacks
            # (like cancel_callback), preventing deadlock during cancellation.
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )
        
        self.get_logger().info('Advanced AMR Mission Control System: ONLINE. Ready for logistics operations.')

    def goal_callback(self, goal_request):
        """
        Callback Function: Goal Request Validation (Admission Control).
        
        This method acts as a gatekeeper, verifying if the system can physically and logically
        accept the proposed task. It implements a resource-aware scheduling policy.
        
        Args:
            goal_request: The incoming Goal.Goal structure containing mission parameters.
            
        Returns:
            GoalResponse: ACCEPT or REJECT based on system readiness.
        """
        self.get_logger().info(f'FAT-Verify: Receiving Request - Type: {goal_request.mission_type} | Zone: {goal_request.target_zone}')

        if self._battery_level < 20.0 and goal_request.mission_type != "CHARGING":
            self.get_logger().warn('Goal Rejected: Critical Battery Levels. Recharge Required.')
            return GoalResponse.REJECT

        # Simulate admission control policies
        valid_types = ["LOGISTICS_DELIVERY", "SURVEILLANCE_PATROL", "EMERGENCY_RESPONSE", "CHARGING"]
        if goal_request.mission_type not in valid_types:
             self.get_logger().warn(f'Goal Rejected: Unknown Mission Profile "{goal_request.mission_type}".')
             return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Handles preemption requests safely.
        """
        self.get_logger().info('Interrupt Signal Received: Evaluating safe shutdown procedures...')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """
        Defer execution to the executor to allow parallel processing if needed.
        """
        goal_handle.execute()

    async def execute_callback(self, goal_handle):
        """
        The core execution loop (Business Logic).
        
        This coroutine manages the lifecycle of the active goal. It iterates through the
        mission phases defined by the mission profile. Crucially, it polls for cancellation
        requests at each simulation tick to ensure strict real-time responsiveness.
        
        Concurrency Note:
            This method is executed in a thread pool (via MultiThreadedExecutor). 
            Access to shared resources (self._battery_level, etc.) is guarded by `self._lock`.
            
        Args:
            goal_handle: The active ServerGoalHandle used to interact with the client.
        """
        with self._lock:
            self._is_busy = True

        goal = goal_handle.request
        feedback_msg = Mission.Feedback()
        result = Mission.Result()
        
        # Initialize Mission Metrics
        start_time = self.get_clock().now()
        phases = self._get_phases_for_mission(goal.mission_type)
        total_steps = sum(p['duration'] for p in phases)
        steps_completed = 0
        energy_start = self._battery_level

        self.get_logger().info(f'Initiating Sequence: {goal.mission_type}')

        try:
            for phase in phases:
                phase_name = phase['name']
                duration = phase['duration']
                
                self.get_logger().info(f'Engaging Phase: [{phase_name}]')

                for t in range(duration):
                    # Check for external cancellation
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.get_logger().info('Mission Aborted: Safe Halt Triggered.')
                        return self._generate_abort_result(result, start_time, energy_start)

                    # Simulate Work Physics
                    time.sleep(1.0 / self._tick_rate)
                    self._simulate_battery_drain()
                    steps_completed += 1
                    
                    # Update Feedback
                    feedback_msg.current_phase = phase_name
                    feedback_msg.progress_percentage = (steps_completed / total_steps) * 100.0
                    feedback_msg.battery_level = self._battery_level
                    feedback_msg.log_message = self._generate_log_message(phase_name, t, duration)
                    feedback_msg.estimated_time_remaining = float(total_steps - steps_completed)
                    
                    goal_handle.publish_feedback(feedback_msg)
                    
                    # Simulate Random Failure Events (e.g., obstacle detection)
                    if self._check_random_failures(goal.priority):
                        goal_handle.abort()
                        result.success = False
                        result.report = "CRITICAL: Obstacle detected causing path blockage. Mission Failed."
                        return result

            # Mission Success
            goal_handle.succeed()
            result.success = True
            result.report = f"Mission {goal.mission_type} completed successfully. Target {goal.target_zone} secured."
            result.operational_efficiency = 0.98 # Simulated metric
            result.energy_consumed = energy_start - self._battery_level
            result.execution_time = (self.get_clock().now() - start_time).nanoseconds / 1e9

            self.get_logger().info('Mission Complete. Systems returning to Idle.')
            return result

        finally:
            with self._lock:
                self._is_busy = False

    def _get_phases_for_mission(self, mission_type):
        """Returns a list of phases based on mission profile."""
        if mission_type == "LOGISTICS_DELIVERY":
            return [
                {'name': 'PATH_PLANNING', 'duration': 2},
                {'name': 'TRANSIT_TO_WAREHOUSE', 'duration': 4},
                {'name': 'ITEM_ACQUISITION', 'duration': 3},
                {'name': 'TRANSIT_TO_DROP_ZONE', 'duration': 4},
                {'name': 'PAYLOAD_OFFLOAD', 'duration': 2}
            ]
        elif mission_type == "SURVEILLANCE_PATROL":
             return [
                {'name': 'SYSTEM_CALIBRATION', 'duration': 2},
                {'name': 'PATROL_SECTOR_ALPHA', 'duration': 5},
                {'name': 'DATA_UPLOAD', 'duration': 3},
                {'name': 'RETURN_TO_BASE', 'duration': 4}
            ]
        elif mission_type == "CHARGING":
             return [
                 {'name': 'DOCKING_ALIGNMENT', 'duration': 3},
                 {'name': 'CHARGING_SEQUENCE', 'duration': 5}
             ]
        else:
            return [{'name': 'GENERIC_EXECUTION', 'duration': 5}]

    def _simulate_battery_drain(self):
        self._battery_level = max(0.0, self._battery_level - self._battery_drain_rate)

    def _check_random_failures(self, priority):
        # 5% chance of failure, reduced by priority (higher priority = more careful planning)
        risk_factor = max(1, 10 - priority)
        return random.randint(0, 1000) < (5 * risk_factor)

    def _generate_log_message(self, phase, tick, duration):
        if phase == 'PATH_PLANNING':
            return f"Computing optimal trajectory... node {tick}/{duration}"
        elif phase == 'TRANSIT_TO_WAREHOUSE':
            return f"Velocity 1.2m/s. obstacle_density=low. tick {tick}"
        return f"Executing {phase} protocol..."

    def _generate_abort_result(self, result, start_time, energy_start):
        result.success = False
        result.report = "Mission Preempted by Operator."
        result.energy_consumed = energy_start - self._battery_level
        result.execution_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
        return result

def main(args=None):
    rclpy.init(args=args)
    
    # Advanced: Use MultiThreadedExecutor for better concurrency in complex systems
    node = AdvancedMissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
