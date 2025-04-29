import rclpy
from typing import List
import rclpy.parameter
import os
import datetime
from system_interfaces.msg import ExperimentLogging

from python_base_class.engel_base_class import ENGELBaseClass
from experiment_setup.config.experiment_logger_config import comm_types

from ament_index_python.packages import get_package_share_directory


class ExperimentLoggerNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List[dict],
        node_name: str = "experiment_logger",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the ExperimentLoggerNode.

        Parameters:
        comm_types (List[dict]): A list containing communication types for the node.
        node_name (str): The name of the node. Default is "experiment_logger_node".
        param_file (str): The name of the parameter file. Default is "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("experiment_setup"), "resources", param_file
        )

        self.csv_write_time_period = None

        super().__init__(node_name, comm_types, config_file)

        self.trigger_configure()
        self.trigger_activate()

        start = os.environ.get("START_TIME")
        scenario = os.environ.get("SCENARIO_FILE").replace(".yaml", "")
        rule_set = os.environ.get("RULE_SET").replace(".txt", "")

        # Define data for logging
        self.columns = [
            "timestamp",
            "source",
            "rule_name",
            "adaptation_type",
            "adaptation_status",
            "success",
            "iou",
            "vehicle_iou",
            "road_iou",
            "gt_failure_name",
            "is_gt_failure",
        ]
        self.logged_data = [",".join(self.columns)]
        self.logged_data.append(",".join([start] * len(self.columns)))
        log_dir = os.path.join("/logs", scenario + "_" + rule_set)
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(
            log_dir,
            datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv",
        )
        self.timer = self.create_timer(self.csv_write_time_period, self._write_to_csv)

    def logger_callback(self, log_msg: ExperimentLogging) -> None:
        """
        Log the incoming message to the logged_data.

        Parameters:
        log_msg (ExperimentLogging): A logging message to be written to the log file.
        """
        new_data = []
        for attr in self.columns:
            value = getattr(log_msg, attr)
            new_data.append(str(value))
        self.logged_data.append(",".join(new_data))

    def _write_to_csv(self) -> None:
        """Write the current logs to a csv file."""

        self.logger.info(f"Wrote file to {self.log_file_path}")
        with open(self.log_file_path, "w") as f:
            f.write("\n".join(self.logged_data))


def main() -> None:
    """
    Main function to initialize and run the ExperimentLoggerNode.
    """
    rclpy.init()
    test = ExperimentLoggerNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
