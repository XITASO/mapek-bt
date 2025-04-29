import rclpy
from typing import List
import os
import sys
import yaml
from system_interfaces.msg import ExperimentLogging
from system_interfaces.srv import SetHeartbeatStatus, SetFlightPhase
import subprocess

from python_base_class.engel_base_class import ENGELBaseClass, CommunicationTypes
from experiment_setup.config.scenario_executor_config import comm_types

from ament_index_python.packages import get_package_share_directory


class ScenarioExecutor(ENGELBaseClass):
    """This node executes a scenario from a given scenario.yaml file.
    Make sure to run it with use_sim_time=True to sync with the rosbag time.
    ros2 run experiment_setup scenario_executor --ros-args --param use_sim_time:=true"""

    def __init__(
        self,
        comm_types: List[dict],
        node_name: str = "scenario_executor",
        param_file: str = "params.yaml",
        scenario_file: str = "",
    ) -> None:
        """
        Initializes the ScenarioExecutor node.

        Parameters:
        comm_types (List[dict]): A list containing communication types for the node.
        node_name (str): The name of the node. Default is "message_drop_node".
        param_file (str): The name of the parameter file. Default is "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("experiment_setup"), "resources", param_file
        )
        self.scenario_file = None
        super().__init__(node_name, comm_types, config_file, namespace="experiment_setup")

        self.trigger_configure()
        self.trigger_activate()

        if scenario_file == "":
            scenario_file = self.scenario_file
            self.get_logger().info(f"Using scenario file from params definition: {scenario_file}")

        self.timer = self.create_timer(0.1, self.clock_callback)

        # Read in all scenario specific events
        scenario_file = os.path.join(
            get_package_share_directory("experiment_setup"),
            "resources",
            "scenarios",
            scenario_file,
        )
        if os.path.exists(scenario_file):
            self.logger.info(f"Loading scenario from {scenario_file}")
            with open(scenario_file, "rb") as f:
                scenario_data = yaml.load(f, Loader=yaml.FullLoader)
            self.ordered_events = sorted(scenario_data, key=lambda x: x["stamp"])
        else:
            self.ordered_events = []

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

    def clock_callback(self):
        """Triggers new events from the scenario event list according to timestamp."""
        stamp = self.get_clock().now().nanoseconds / 1e9
        if len(self.ordered_events) == 0:
            return

        while stamp >= self.ordered_events[0]["stamp"]:
            event = self.ordered_events.pop(0)

            # Check for the corresponding action for the event type
            if event["type"] == "parameter_adaptation":
                self._adapt_parameters(event)
            elif event["type"] == "lifecycle_change":
                self._change_lifecycle_state(event)
            elif event["type"] == "degrade_heartbeat":
                self._degrade_heartbeat(event)
            elif event["type"] == "change_flightphase":
                self._change_flightphase(event)
            else:
                self.logger.warn(f"Unknown event type {event['type']}")
                self.logger.info(f"Time: ")

            # stop if list is empty
            if len(self.ordered_events) == 0:
                return

    def _adapt_parameters(self, event: dict) -> None:
        """Set a new parameter according to the event.
        
        Parameters:
        event (dict): A dict containing the specific values for an event.
        """

        # test if opening new process works better
        process = subprocess.Popen(
            args=[
                "ros2",
                "param",
                "set",
                event["target_node"],
                event["name"],
                str(event["value"]),
            ]
        )

        self.logger.info(
            f"New scenario adaptation: {event['name']} with failure state {event['is_gt_failure']}"
        )

        # Log the expected ground truth of the system state
        publisher = self.get_comm_object("/scenario_executor_log")
        log_msg = ExperimentLogging(
            timestamp=self.get_clock().now().nanoseconds,
            source="/scenario_executor_log",
            gt_failure_name=event["name"],
            is_gt_failure=event["is_gt_failure"],
        )
        publisher.publish(log_msg)

    def _change_lifecycle_state(self, event: dict):
        """
        Change the live cycle state of the target node.
        
        Parameters:
        event (dict): A dict containing the specific values for an event.
        """

        # No feedback needed, as the on_shutdown function will take care of the exact logging time and success tracking
        subprocess.Popen(
            ["ros2", "lifecycle", "set", event["target_node"], event["state"]],
            stdout=subprocess.PIPE,
        )
        self.logger.info(
            f"New scenario adaptation: {event['target_node']} switching to {event['state']}"
        )

    def _degrade_heartbeat(self, event: dict) -> None:
        """
        Degrade the heartbeat of a node for test purposes.
        
        Parameters:
        event (dict): A dict containing the specific values for an event.
        """
        service_cient = self.get_comm_object(
            event["target_service"], comm_type=CommunicationTypes.SERVICE_CLIENT
        )
        while not service_cient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        request = SetHeartbeatStatus.Request()
        request.status = event["value"]
        future = service_cient.call_async(request)


    def _change_flightphase(self, event: dict):
        """
        Change the flightphase to a new phase.
        
        Parameters:
        event (dict): A dict containing the specific values for an event.
        """
        service_cient = self.get_comm_object(
            "/set_flight_phase", comm_type=CommunicationTypes.SERVICE_CLIENT
        )
        while not service_cient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        request = SetFlightPhase.Request()
        request.flightphase = event["value"]
        future = service_cient.call_async(request)

        self.logger.info(
            f"New scenario adaptation: Flightphase changed to {event['value']}."
        )


def main() -> None:
    """
    Main function to initialize and run the ScenarioExecutor.
    """
    rclpy.init()

    scenario_file = ""
    if len(sys.argv) > 1:
        print(f"Using scenario file {sys.argv[1]}")
        scenario_file = sys.argv[1]

    test = ScenarioExecutor(comm_types=comm_types, scenario_file=scenario_file)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
