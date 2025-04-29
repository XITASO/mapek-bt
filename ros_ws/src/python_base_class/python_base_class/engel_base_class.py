from rclpy.lifecycle import State, TransitionCallbackReturn, Node
from rclpy.parameter import Parameter
from rclpy.timer import Timer
from python_base_class.node_config import CommunicationTypes
from typing import List, Dict, Any
from system_interfaces.msg import Heartbeat, ExperimentLogging
from system_interfaces.srv import SetLifecycleChanges
from copy import deepcopy
from lifecycle_msgs.msg import Transition, State as LcState

from rcl_interfaces.msg import SetParametersResult
import yaml
from rclpy.qos import QoSProfile


class ENGELBaseClass(Node):
    def __init__(self, node_name, comm_config: List, config_file: str, **kwargs) -> None:
        """
        Initializes the ENGELBaseClass node.

        Parameters:
        node_name (str): The name of the ROS node.
        comm_config (List): A list of dictionaries containing configuration details for various communication types.
        config_file(str): Path to the file describing the parameters for all nodes.
        **kwargs: Additional keyword arguments to be passed to the parent class constructor.
        """
        super().__init__(node_name, **kwargs)
        self.comm_objects = deepcopy(comm_config)
        self.ns = "" if self.get_namespace() == "/" else self.get_namespace()
        self.comm_objects.append(
            {
                "comm_type": CommunicationTypes.PUBLISHER,
                "name": f"/{node_name}/heartbeat",
                "msg_type": Heartbeat,
                "time_period": 1 / 20,
                "callback": "heartbeat_callback",
                "always_on": True,
            }
        )
        self.comm_objects.append(
            {
                "comm_type": CommunicationTypes.PUBLISHER,
                "name": f"/{node_name}/experiment_log",
                "msg_type": ExperimentLogging,
                "always_on": False,
            }
        )
        self.comm_objects.append(
         {
             "comm_type": CommunicationTypes.SERVICE,
             "name": f"/{node_name}/set_lifecycle_changes",
             "srv_type": SetLifecycleChanges,
             "callback": "handle_set_lifecycle_changes",
             "always_on": True,
         }
        ) 

        self.heartbeat_msg = Heartbeat()
        self.heartbeat_msg.status = Heartbeat.HB_STATUS_OK
        self.lc_state = LcState.PRIMARY_STATE_UNCONFIGURED

        # Timer for handling lc related events
        self.lc_timer = self.create_timer(timer_period_sec=0.1, callback=self.check_lc_state)

        # Define the required keys for each communication type
        self.required_keys = {
            CommunicationTypes.PUBLISHER: [
                "name",
                "msg_type",
                "always_on",
            ],
            CommunicationTypes.SUBSCRIPTION: [
                "name",
                "msg_type",
                "callback",
                "always_on",
            ],
            CommunicationTypes.SERVICE: ["name", "srv_type", "callback", "always_on"],
            CommunicationTypes.SERVICE_CLIENT: ["name", "srv_type", "always_on"],
        }
        self.activate = {
            CommunicationTypes.PUBLISHER: self._register_publisher,
            CommunicationTypes.SUBSCRIPTION: self._register_subscription,
            CommunicationTypes.SERVICE: self._register_service,
            CommunicationTypes.SERVICE_CLIENT: self._register_service_client,
        }
        self.destroy = {
            CommunicationTypes.PUBLISHER: self.destroy_publisher,
            CommunicationTypes.SUBSCRIPTION: self.destroy_subscription,
            CommunicationTypes.SERVICE: self.destroy_service,
            CommunicationTypes.SERVICE_CLIENT: self.destroy_client,
        }

        self.trigger_transition = {
            Transition.TRANSITION_ACTIVATE: self.trigger_activate,
            Transition.TRANSITION_CONFIGURE: self.trigger_configure,
            Transition.TRANSITION_DEACTIVATE: self.trigger_deactivate,
        }

        self.logger = self.get_logger()
        self.initialize_parameters(config_file=config_file)
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    def set_heartbeat_status(self, status: int):
        """
        Sets the status of the heartbeat message.
        Valid values are defined in the Heartbeat msg

        Parameters:
        status (int): The new status to set for the heartbeat message.
        """
        assert status == Heartbeat.HB_STATUS_OK or status == Heartbeat.HB_STATUS_DEGRADED or status == Heartbeat.HB_STATUS_FAILURE, "Invalid status set for heartbeat message"
        self.heartbeat_msg.status = status

    def heartbeat_callback(self):
        """
        Publishes the heartbeat message using the specified publisher.

        This method retrieves the publisher associated with the heartbeat topic and publishes the current
        heartbeat message to it.
        """
        publisher = self.get_comm_object(f"/{self.get_name()}/heartbeat")
        self.heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        self.heartbeat_msg.header.frame_id = f"/{self.get_name()}/heartbeat"
        self.heartbeat_msg.lc_state = self.lc_state
        publisher.publish(self.heartbeat_msg)

    def execute_transition(self, transition: Transition) -> bool:
        """
        Executes a lifecycle transition.

        Parameters:
        transition (Transition): The transition to execute.

        Returns:
        bool: True if the transition was successful, False otherwise.
        """

        try:
            self.trigger_transition[transition.id]()
            self.get_logger().info(f"Transition {transition.id} succeeded")
            return True
        except:
            self.get_logger().error(f"Transition {transition.id} failed")
            return False

    def handle_set_lifecycle_changes(self, request: SetLifecycleChanges.Request, response: SetLifecycleChanges.Response):
        """
        Callback for the SetLifecycleChanges service.

        Parameters:
        request: The request message containing the transitions.
        response: The response message to be filled with the results.

        """
        for transition in request.transitions:
            success = self.execute_transition(transition)
            response.results.append(success)
        return response

    def get_comm_object(
        self,
        topic_name: str,
        comm_type: CommunicationTypes = CommunicationTypes.PUBLISHER,
        param: str = None,
    ) -> Dict:
        """
        Retrieves a publisher object by its topic name.

        Parameters:
        topic_name (str): The name of the topic associated with the publisher.
        comm_type (CommunicationTypes): type of communication. Can be either publisher, subscriber, service client or service
        param (str): parameter describing the topic a comm type, e.g. subscription listens to

        Returns:
        The communication object that belongs to the topic / service name
        """

        def find_comm_type_by_name(name: str):
            return next(
                (
                    item["obj"]
                    for item in self.comm_objects
                    if item["name"] == name and item["comm_type"] == comm_type
                ),
                None,
            )

        return find_comm_type_by_name(topic_name)

    def find_comm_type_by_param(self, param: str) -> Dict:
        return next(
            (item for item in self.comm_objects if "param" in item and item["param"] == param),
            None,
        )

    def _validate_comm_type(self, comm_type_dict: Dict[str, Any]) -> bool:
        """
        Validates if the input dictionary has the required keys for the given communication type.

        This method performs the following checks:
        1. Ensures that the 'comm_type' key exists and its value is an instance of CommunicationTypes.
        2. Retrieves the list of required keys for the specified communication type.
        3. Verifies that all required keys are present in the input dictionary.

        Parameters:
        comm_type_dict (Dict[str, Any]): A dictionary containing the configuration for a communication type.

        Returns:
        bool: True if the input dictionary contains all required keys for the specified communication type, False otherwise.

        Raises:
        ValueError: If 'comm_type' is not an instance of CommunicationTypes or if the communication type is unknown.
        Exception: Catches any other exceptions that may occur during validation and logs an error message.
        """
        try:
            comm_type = comm_type_dict.get("comm_type")
            if not isinstance(comm_type, CommunicationTypes):
                raise ValueError(
                    "Invalid 'comm_type'. It must be an instance of CommunicationTypes."
                )

            # Get the required keys for the given communication type
            required_keys_for_type = self.required_keys.get(comm_type)
            if required_keys_for_type is None:
                raise ValueError(f"Unknown communication type: {comm_type}")

            # Check if all required keys are present in the input dictionary
            for key in required_keys_for_type:
                if key not in comm_type_dict:
                    self.get_logger().error(
                        f"Missing required key: {key} for communication type: {comm_type}"
                    )
                    return False

            # All checks passed
            return True

        except Exception as e:
            print(f"Validation Error: {e}")
            return False

    def _activate_communication_objects(self, activating: bool):
        """
        Activates communication objects according to their configuration.

        This method iterates over the list of communication objects and validates their configuration.
        If valid, it activates the communication object using the appropriate registration method.
        Parameters:
            activating (bool): Indicates wether we are transitioning from unconfigured to inactive or
                                from inactive to active

        Raises:
        ValueError: If a communication object's configuration is not valid.
        """
        for element in self.comm_objects:
            if (not activating and element['always_on'] == True) or activating:
                if self._validate_comm_type(element):
                    if 'callback' in element and not hasattr(self, element['callback']):
                        self.logger.error(f"Callback specified for {element['name']} but not implemented by the class")
                        continue
                    if not 'obj' in element:
                        element = self.activate[element["comm_type"]](element)
                else:
                    self.logger.error(f"{element} is not a valid configuration of a communication type")

    def validate_comm_name(self, comm_name: str) -> str:
        if comm_name[0] == "/":
            return comm_name
        else:
            return f"/{comm_name}"

    def _register_publisher(self, publisher: Dict):
        """
        Registers a ROS publisher based on the provided configuration.

        Parameters:
        publisher (Dict): A dictionary containing the configuration for the publisher.

        Returns:
        Dict: The modified publisher dictionary including the created publisher object and timer.
        """
        topic_name = self.validate_comm_name(publisher["name"])
        qos_profile = QoSProfile(depth=publisher["q_size"]) if "q_size" in publisher else 10
        publisher["obj"] = self.create_publisher(
            publisher["msg_type"], f"{self.ns}{topic_name}", qos_profile=qos_profile
        )
        self.get_logger().info(f"Topic name: {publisher['obj'].topic_name}")

        if "time_period" in publisher:
            publisher["timer"] = self.create_timer(
                publisher["time_period"],
                self.__getattribute__(publisher["callback"]),
            )
        self.get_logger().info(f"Created publisher on topic {publisher['name']}")
        return publisher

    def _register_subscription(self, subscription: Dict):
        """
        Registers a ROS subscription based on the provided configuration.

        Parameters:
        subscription (Dict): A dictionary containing the configuration for the subscription.

        Returns:
        Dict: The modified subscription dictionary including the created subscription object.
        """
        if 'param' in subscription and not 'initialized' in subscription and self.params:
            self.logger.info(f"Parameter for subscription {subscription['name']} defined. Will use the topic name from the parameter associated here")
            topic_name = self.validate_comm_name(self.params[subscription['param']])
        else:
            topic_name = self.validate_comm_name(subscription["name"])
        subscription["obj"] = self.create_subscription(
            msg_type=subscription["msg_type"],
            topic=f"{self.ns}{topic_name}",
            callback=self.__getattribute__(subscription["callback"]),
            qos_profile=10,
        )
        subscription['initialized'] = True
        self.get_logger().info(f"Created subscription on topic {topic_name}")
        return subscription

    def _register_service(self, service: Dict):
        """
        Registers a ROS service based on the provided configuration.

        Parameters:
        service (Dict): A dictionary containing the configuration for the service.

        Returns:
        Dict: The modified service dictionary including the created service object.
        """
        service_name = self.validate_comm_name(service["name"])
        service["obj"] = self.create_service(
            srv_type=service["srv_type"],
            srv_name=f"{self.ns}{service_name}",
            callback=self.__getattribute__(service["callback"]),
        )
        self.get_logger().info(f"Created service with name {service['name']}")
        return service

    def _register_service_client(self, client: Dict):
        """
        Registers a ROS service client based on the provided configuration.

        Parameters:
        client (Dict): A dictionary containing the configuration for the service client.

        Returns:
        Dict: The modified client dictionary including the created service client object.
        """
        service_name = self.validate_comm_name(client["name"])
        client["obj"] = self.create_client(
            srv_type=client["srv_type"], srv_name=f"{self.ns}{service_name}"
        )
        self.get_logger().info(f"Created client for {client['name']}")
        return client

    def _register_actions(self):
        """
        Placeholder for registering action servers. Currently not implemented.

        This method logs a warning message indicating that the registration of action servers is not implemented.
        """
        self.get_logger().warning("Registering action servers is currently not implemented")

    def _register_action_clients(self):
        """
        Placeholder for registering action clients. Currently not implemented.
        """
        self.get_logger().warning("Registering action clients is currently not implemented")

    def _unregister_communication(self, force: bool = False):
        """
        Unregisters communication objects based on their configuration.

        This method iterates over the list of communication objects and destroys their associated ROS entities.
        If `force` is True, it will destroy all communication objects, otherwise, it will only destroy those
        that are not marked as 'always_on'.

        Parameters:
        force (bool): If True, forces the destruction of all communication objects regardless of 'always_on'. Defaults to False.
        """
        for comm_obj in self.comm_objects:
            if comm_obj["always_on"] is False or force is True:
                self.destroy[comm_obj["comm_type"]](comm_obj["obj"])
                comm_obj.pop("obj", None)
                if comm_obj["comm_type"] == CommunicationTypes.PUBLISHER and "timer" in comm_obj:
                    self.destroy_timer(comm_obj["timer"])

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback for when the node enters the 'configuring' state.

        :param state: The current state of the node
        :return: The result of the transition
        """
        self.logger.debug("I am configured now")
        self._activate_communication_objects(activating=False)

        self.lc_state = LcState.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback for when the node enters the 'activating' state.

        :param state: The current state of the node
        :return: The result of the transition
        """
        self._activate_communication_objects(activating=True)

        self.logger.debug("I am activated now")

        self.lc_state = LcState.PRIMARY_STATE_ACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback for when the node enters the 'deactivating' state.

        :param state: The current state of the node
        :return: The result of the transition
        """
        self._unregister_communication()

        self.lc_state = LcState.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle callback for when the node enters the 'shutting down' state.

        :param state: The current state of the node
        :return: The result of the transition
        """
        self.publish_experiment_log("shutdown")
        self._unregister_communication(force=True)
        self.logger.info("shutting down...")

        self.lc_state = LcState.PRIMARY_STATE_FINALIZED
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        """
        Lifecycle callback for when the node enters the 'error' state.

        :param state: The current state of the node
        :return: The result of the transition
        """
        self.lc_state = LcState.PRIMARY_STATE_UNCONFIGURED
        return super().on_error(state)

    def initialize_parameters(self, config_file: str):
        """
        Initializes the node's parameters from a configuration file.

        Parameters:
        config_file (str): The path to the YAML configuration file containing the parameters.

        This method reads the configuration file, extracts the parameters specific to the node's name,
        and declares these parameters in the node.
        """
        try:
            with open(config_file, "r") as f:
                self.params = yaml.load(f, Loader=yaml.FullLoader)[self.get_name()]["ros__parameters"]
        except:
            self.logger.warning("No parameters defined for this node, doesn't have to be bad")
            return

        if self.params:
            for key, value in self.params.items():
                try:
                    # The following line will throw an exception if the class variable does not exist
                    # If it exists, we can set the attribute
                    getattr(self, key)
                    setattr(self, key, value)
                except:
                    self.logger.warning(f"For param {key}, there is no class member existing")
            self.declare_parameters(namespace=None, parameters=tuple(self.params.items()))
        else:
            self.logger.warning("No parameters specified for this node")

    def validate_parameters(self) -> bool:
        """
        Validates whether all defined parameters exist as attributes in the class instance.

        This method iterates through the list of defined parameters (`self.params`) and checks
        if each parameter is present as an attribute in the class instance. If any parameter is missing,
        the method sets `all_params_exist` to False and continues checking the remaining parameters.

        Returns:
        bool: True if all defined parameters exist as attributes, False otherwise.
        """
        if not self.params:
            return True

        all_params_exist = True
        for defined_param in self.params:
            if not hasattr(self, defined_param):
                all_params_exist = False

        return all_params_exist

    def parameter_change_callback(self, params: List) -> SetParametersResult:
        """
        Callback function to handle changes in parameters.

        This method processes a list of parameter changes. For each parameter, it checks if the parameter name
        contains 'topic' or 'service'. If it does, the method calls `handle_comm_change` to manage the change.
        Otherwise, it calls `handle_parameter_value_change` to handle the change.

        If any parameter change fails, the method stops processing further changes and returns a failure result.

        Parameters:
        params (List[Parameter]): A list of parameters that have changed.

        Returns:
        SetParametersResult: An object indicating whether the parameter changes were successful.
        """
        successful = True
        for param in params:
            if "topic" in param.name or "service" in param.name:
                successful = self.handle_comm_change(param)
            else:
                successful = self.handle_parameter_value_change(param)

            if not successful:
                # TODO known issue: If a list of parameters comes and only one of them would fail, why stopping here?
                # Not handling this case right now, as this case should not come up
                break

        # Return success message to indicate the parameters were set successfully
        return SetParametersResult(successful=successful)

    def handle_comm_change(self, param: Parameter) -> bool:
        """Handles changes in communication parameters.

        This method processes a change in a communication parameter. It first updates the parameter value
        using `handle_parameter_value_change`. Then, it finds the corresponding communication type by the
        parameter name, destroys the existing communication object, updates the name of the communication
        object with the new parameter value, and reactivates the communication object.

        Args:
            param (Parameter): The parameter that has changed.

        Returns:
        bool: True if the communication change was successful, False otherwise.
        """
        self.handle_parameter_value_change(param)
        success = True
        item = self.find_comm_type_by_param(param.name)
        if item is None:
            self.logger.warn(f"Could not change the communicaton for {param}")
            return False
        self.destroy[item["comm_type"]](item["obj"])
        item["name"] = param.value
        item = self.activate[item["comm_type"]](item)

        self.logger.debug(f"Handling {param} comm change was successfull: {success}")
        return success

    def handle_parameter_value_change(self, param: Parameter) -> bool:
        """
        Handles a change in the value of a parameter.

        This method checks if the parameter exists as an attribute in the class instance.
        If it does, it updates the attribute with the new value from the parameter.
        If the parameter does not exist, it sets the success flag to False.

        Parameters:
        param (Parameter): The parameter whose value has changed.

        Returns:
        bool: True if the parameter value was successfully updated, False otherwise.
        """
        success = True
        if hasattr(self, param.name):
            setattr(self, param.name, param.value)
        else:
            success = False
        self.logger.debug(f"Handling {param} value change was successfull: {success}")
        return success
    

    def check_lc_state(self) -> None:
        """React to changes in the life cycle state. Note that this is triggered with a timer and not only on change compared to functions like on_shutdown."""
        if self.lc_state == LcState.PRIMARY_STATE_FINALIZED:
            exit()
    

    def publish_experiment_log(self, lifecycle_state: str) -> None:
        publisher = self.get_comm_object(f"/{self.get_name()}/experiment_log", CommunicationTypes.PUBLISHER)
        log_msg = ExperimentLogging(
            timestamp=self.get_clock().now().nanoseconds,
            source=f"/{self.get_name()}/experiment_log",
            gt_failure_name=lifecycle_state,
            is_gt_failure=True,
        )
        publisher.publish(log_msg)
