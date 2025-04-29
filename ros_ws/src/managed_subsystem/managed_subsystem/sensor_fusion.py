import rclpy
from typing import List
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from system_interfaces.msg import SensorFusion
from system_interfaces.msg import FusionType
import os

from python_base_class.node_config import CommunicationTypes
from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.sensor_fusion_config import comm_types

from ament_index_python.packages import get_package_share_directory


class SensorFusionNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List,
        node_name: str = "sensor_fusion",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the SensorFusionNode class.

        Parameters:
        comm_types (List): A list containing the communication types configuration.
        node_name (str): The name of the node. Defaults to "sensor_fusion_node".
        param_file (str): The parameter file path. Defaults to "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        self.topic_camera_input = None 
        self.modality = None
        self.delta_t_threshold = None
        super().__init__(node_name, comm_types, config_file)

        self.trigger_configure()

        self.data_buffer = {
            "depth": [Image(), -1],
            "rgb": [Image(), -1],
        }

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

    def rgb_callback(self, image: Image) -> None:
        """
        Callback function for RGB image data.

        Parameters:
        image (Image): The RGB image message.
        """
        self.add_to_data_buffer(image, "rgb")

    def depth_callback(self, image: Image) -> None:
        """
        Callback function for depth image data.

        Parameters:
        image (Image): The depth image message.
        """
        self.add_to_data_buffer(image, "depth")

    def add_to_data_buffer(self, image: Image, key: str) -> None:
        """
        Adds incoming image data to the data buffer.

        Parameters:
        image (Image): The image message.
        key (str): The data type key ('rgb' or 'depth').
        """
        time_code = image.header.stamp.sec + image.header.stamp.nanosec / 1e9
        self.data_buffer[key] = [image, time_code]

        if self.modality == FusionType.FUSION:
            if self.check_for_full_batch():
                self.publish_fused_data()
        else:
            self.publish_fused_data()

    def check_for_full_batch(self) -> bool:
        """
        Checks if there is almost the same time stamp on both data sources.
        True by default, if only one data source is used.

        Returns:
        bool: True if the time difference is within the threshold, False otherwise.
        """
        if self.modality == FusionType.FUSION:
            return (
                abs(self.data_buffer["rgb"][1] - self.data_buffer["depth"][1])
                <= self.delta_t_threshold
            )
        return True

    def publish_fused_data(self) -> None:
        """
        Publishes the fused sensor data.
        """
        header = Header()
        header.frame_id = "/sensors_fused"
        if self.modality == FusionType.DEPTH:
            header.stamp = self.data_buffer["depth"][0].header.stamp
        else:
            header.stamp = self.data_buffer["rgb"][0].header.stamp

        out_msg = SensorFusion(
            depth=self.data_buffer["depth"][0],
            rgb=self.data_buffer["rgb"][0],
            modality=FusionType(fusion_type=self.modality),
            header=header,
        )

        publisher = self.get_comm_object("/sensors_fused", comm_type=CommunicationTypes.PUBLISHER)
        publisher.publish(out_msg)


def main() -> None:
    """
    Main function to initiate the SensorFusionNode.
    """
    rclpy.init()
    test = SensorFusionNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
