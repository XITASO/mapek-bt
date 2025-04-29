import rclpy
from typing import List, Dict, Optional
import rclpy.logging
import rclpy.parameter
from sensor_msgs.msg import Image
import numpy as np
import os
from cv_bridge import CvBridge
from managed_subsystem.utils.segmentation_logic import UAVidSegmenter
from system_interfaces.msg import SegmentationMetrics, ExperimentLogging

from python_base_class.engel_base_class import ENGELBaseClass
from experiment_setup.config.evaluator_config import comm_types

from ament_index_python.packages import get_package_share_directory
import torch


class EvaluatorNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "evaluator",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the EvaluatorNode with the given communication types and parameters.

        Parameters:
        comm_types (List): List of communication types.
        node_name (str): Name of the node. Defaults to "evaluator".
        param_file (str): Parameter file name. Defaults to "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("experiment_setup"), "resources", param_file
        )
        self.delta_t_threshold: Optional[float] = None
        self.max_buffer_length: Optional[int] = None
        super().__init__(node_name, comm_types, config_file)

        self.trigger_configure()
        self.trigger_activate()

        if self.validate_parameters() is False:
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

        self.buffer = {"gt": [], "pred": []}
        self.bridge = CvBridge()
        self.segmenter = UAVidSegmenter()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

    def gt_callback(self, msg: Image) -> None:
        self._add_to_buffer("gt", msg)

    def pred_callback(self, msg: Image) -> None:
        self._add_to_buffer("pred", msg)

    def _add_to_buffer(self, key: str, msg: Image) -> None:
        """
        Adds the image message to the appropriate buffer and checks for paired messages.

        Parameters:
        key (str): The key of the buffer to add the message to, either "gt" or "pred".
        msg (Image): The image message to add to the buffer.
        """
        if len(self.buffer[key]) > self.max_buffer_length:
            del self.buffer[key][0]
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.buffer[key].append([msg, timestamp])

        paired_key = "pred" if key == "gt" else "gt"
        paired_msg = self._check_buffer_for_pairs(paired_key, timestamp)

        if paired_msg is not None:
            data = {paired_key: paired_msg, key: msg}
            iou_dict = self._calculate_metrics(**data)

            self._publish_results(msg, iou_dict)

    def _check_buffer_for_pairs(self, key: str, timestamp: float) -> Optional[Image]:
        """
        Checks the buffer for messages paired by timestamp with the given threshold.

        Parameters:
        key (str): The key of the buffer to check for pairs, either "gt" or "pred".
        timestamp (float): The timestamp to match messages.

        Returns:
        Optional[Image]: The paired message if found, otherwise None.
        """
        if len(self.buffer[key]) == 0:
            return None
        delta_ts = np.abs(np.array([v[1] for v in self.buffer[key]]) - timestamp)
        if np.min(delta_ts) < self.delta_t_threshold:
            idx = np.argmin(delta_ts)
            return self.buffer[key].pop(idx)[0]
        return None

    def _calculate_metrics(self, gt: Image, pred: Image) -> Dict[str, float]:
        """
        Calculates segmentation metrics using ground truth and predicted images.

        Parameters:
        gt (Image): The ground truth image message.
        pred (Image): The predicted image message.

        Returns:
        Dict[str, float]: A dictionary containing vehicle, road, and overall IoU scores.
        """
        gt_tensor = (
            self.segmenter.prepare_mlb(self.bridge.imgmsg_to_cv2(gt))
            .detach()
            .to(self.device)
        )
        pred_tensor = (
            torch.tensor(self.bridge.imgmsg_to_cv2(pred)).detach().to(self.device)
        )

        class_iou = self.segmenter.calc_metrics(pred_tensor, gt_tensor)
        vehicle_classes = [
            "Rider",
            "Car",
            "Truck",
            "Bus",
            "Train",
            "Motorcycle",
            "Bicycle",
        ]

        vehicle_iou = float(np.nanmean(np.array([class_iou[key] for key in vehicle_classes])))
        overall_iou = float(np.nanmean(np.array([value for value in class_iou.values()])))

        return {"vehicle_iou": vehicle_iou, "road_iou": float(class_iou["Road"]), "overall_iou": overall_iou}

    def _publish_results(self, msg: Image, iou_dict: Dict[str, float]) -> None:
        """
        Publishes results and logs them.

        Parameters:
        msg (Image): The image message associated with the metrics.
        iou_dict (Dict[str, float]): A dictionary containing IoU scores.
        """
        publisher = self.get_comm_object("/seg_metrics")
        metric_msg = SegmentationMetrics()
        metric_msg.header = msg.header
        metric_msg.header.frame_id = "/seg_metrics"
        metric_msg.mean_iou = iou_dict["overall_iou"]
        publisher.publish(metric_msg)

        publisher = self.get_comm_object("/evaluator_log")
        log_msg = ExperimentLogging(
            timestamp=self.get_clock().now().nanoseconds,
            source="/evaluator_log",
            iou=iou_dict["overall_iou"],
            vehicle_iou=iou_dict["vehicle_iou"],
            road_iou=iou_dict["road_iou"]
        )
        publisher.publish(log_msg)


def main() -> None:
    """
    Main function to initialize and run the EvaluatorNode.
    """
    rclpy.init()
    test = EvaluatorNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()