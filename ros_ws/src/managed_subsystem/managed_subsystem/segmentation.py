import rclpy
from typing import List
import rclpy.logging
from system_interfaces.msg import SensorFusion, FusionType
from cv_bridge import CvBridge
import os

from python_base_class.node_config import CommunicationTypes
from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.segmentation_config import comm_types
from managed_subsystem.utils.segmentation_logic import UAVidSegmenter

from ament_index_python.packages import get_package_share_directory


class SegmentationNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "segmentation",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the SegmentationNode.

        Parameters:
        comm_types (List): A list of communication configurations.
        node_name (str): The name of the ROS node. Defaults to "segmentation_node".
        param_file (str): The configuration file containing parameters, located in the package resources. Defaults to "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        super().__init__(node_name, comm_types, config_file)

        self.trigger_configure()

        self.segmenter = UAVidSegmenter(checkpoints_path=".data/checkpoints", logger=self.logger)
        self.bridge = CvBridge()

        self.map_modality2string = {
            0: "fusion",
            1: "rgb",
            2: "depth",
        }

    def fusion_callback(self, fusion_data: SensorFusion) -> None:
        """
        Callback function for processing sensor fusion data.

        Parameters:
        fusion_data (SensorFusion): The ROS message containing sensor fusion data.
        """
        # In try statement, as image encoding is sometimes not recognized on first image during startup. Will skip these.
        try:
            depth_image, rgb_image = None, None
            if (
                fusion_data.modality.fusion_type == FusionType.DEPTH
                or fusion_data.modality.fusion_type == FusionType.FUSION
            ):
                depth_image = self.bridge.imgmsg_to_cv2(fusion_data.depth, desired_encoding="16UC1")
            if (
                fusion_data.modality.fusion_type == FusionType.RGB
                or fusion_data.modality.fusion_type == FusionType.FUSION
            ):
                rgb_image = self.bridge.imgmsg_to_cv2(fusion_data.rgb, desired_encoding="bgr8")
            data = {"rgb": rgb_image, "depth": depth_image}
        except Exception as e:
            self.logger.info(f"Error processing fusion data: {e}")
            return

        segmentation = self.segmenter.inference(
            data, modality=self.map_modality2string[fusion_data.modality.fusion_type]
        )

        seg_msg = self.bridge.cv2_to_imgmsg(segmentation, encoding="8UC1")
        seg_msg.header = fusion_data.header
        seg_msg.header.frame_id = "/segmentation"

        publisher = self.get_comm_object("/segmentation", comm_type=CommunicationTypes.PUBLISHER)
        publisher.publish(seg_msg)


def main() -> None:
    """
    The main function to initialize and spin the SegmentationNode.
    """
    rclpy.init()
    test = SegmentationNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
