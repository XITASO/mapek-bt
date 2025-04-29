import rclpy
from typing import List
import rclpy.logging
from sensor_msgs.msg import Image
import os
import numpy as np
from cv_bridge import CvBridge

from python_base_class.node_config import CommunicationTypes
from python_base_class.engel_base_class import ENGELBaseClass
from managed_subsystem.config.image_enhancement_config import comm_types

from ament_index_python.packages import get_package_share_directory


class ImageEnhancementNode(ENGELBaseClass):
    def __init__(
        self,
        comm_types: List[dict],
        node_name: str = "image_enhancement",
        param_file: str = "params.yaml",
    ) -> None:
        """
        Initializes the ImageEnhancementNode.

        Parameters:
        comm_types (List[dict]): A list containing communication types for the node.
        node_name (str): The name of the node. Default is "image_enhancement_node".
        param_file (str): The name of the parameter file. Default is "params.yaml".
        """
        config_file = os.path.join(
            get_package_share_directory("managed_subsystem"), "resources", param_file
        )
        super().__init__(node_name, comm_types, config_file)

        self.trigger_configure()

        self.bridge = CvBridge()

        if not self.validate_parameters():
            self.logger.warn(
                f"Not all parameters are initialized correctly. Every parameter in \
                    the params.yaml file has to be a class member of {self.get_name()}"
            )

    def image_callback(self, msg: Image) -> None:
        """
        Callback function for processing received images.

        This method normalizes the incoming image and republishes it on the `/rgb_enhanced` topic.

        Parameters:
        msg (Image): The incoming image message.
        """
        msg = self._normalize_image(msg)
        publisher = self.get_comm_object("/rgb_enhanced", comm_type=CommunicationTypes.PUBLISHER)
        msg.header.frame_id = "/rgb_enhanced"
        publisher.publish(msg)

    def _normalize_image(self, msg: Image) -> Image:
        """
        Normalizes the image with respect to the expected dataset mean and standard deviation.

        This method rudimentarily reverses possible degradation by adjusting color channels.

        Parameters:
        msg (Image): The original image message to be normalized.

        Returns:
        Image: The normalized image message.
        """
        image = self.bridge.imgmsg_to_cv2(msg)

        target_mean = np.array([114.802475, 134.91641386, 136.51041609])
        target_std = np.array([53.58526014, 55.07818607, 59.36811692])

        # Compute the current mean and std of the image for each channel and normalize
        current_mean = image.mean(axis=(0, 1))
        current_std = image.std(axis=(0, 1))
        normalized_image = (image - current_mean) / current_std

        # Scale and shift to the target mean and std
        transformed_image = normalized_image * target_std + target_mean
        transformed_image = np.clip(transformed_image, 0, 255)
        transformed_image = transformed_image.astype(np.uint8)

        msg_new = self.bridge.cv2_to_imgmsg(transformed_image, encoding=msg.encoding)
        msg_new.header = msg.header
        return msg_new


def main() -> None:
    """
    Main function to initialize and run the ImageEnhancementNode.
    """
    rclpy.init()
    test = ImageEnhancementNode(comm_types=comm_types)
    rclpy.spin(test)


if __name__ == "__main__":
    main()
