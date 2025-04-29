from enum import Enum
import math
import time
import sys

from system_interfaces.msg import Flightphase
from system_interfaces.msg import Flightstate

from system_interfaces.srv import SetFlightPhase

import rclpy
from rclpy.node import Node


class FlightphaseEnum(Enum):
    NOT_AVAILABLE = 0
    CRUISE = 1
    DESCENT = 2
    APPROACH = 3
    FINAL_APPROACH = 4
    LANDING = 5

def flightphaseToString(flightphase_enum:FlightphaseEnum)->str:
    """
    It does what you think it does.

    Parameters:
    flightphase(FlightphaseEnum): flightphase as enum

    Returns:
    flightphase as string 
    """
    if flightphase_enum == FlightphaseEnum.NOT_AVAILABLE:
        return "not_available"
    if flightphase_enum == FlightphaseEnum.CRUISE:
        return "cruise"
    if flightphase_enum == FlightphaseEnum.DESCENT:
        return "descent"
    if flightphase_enum == FlightphaseEnum.APPROACH:
        return "approach"
    if flightphase_enum == FlightphaseEnum.FINAL_APPROACH:
        return "final_approach"
    if flightphase_enum == FlightphaseEnum.LANDING:
        return "landing"
    return "not_available"

class TriggerCondition:

    def __init__(
        self, flightphase_FROM : FlightphaseEnum, flightphase_TO : FlightphaseEnum,
        trigger_dist=1e6, trigger_hae=1e6, trigger_agl=1e6
    ) -> None:
        """
        Initializes the TriggerClass.

        Parameters:
        flightphase_FROM(FlightphaseEnum): the flightphase we want to switch from
        flightphase_TO(FlightphaseEnum): the flightphase we want to switch to
        trigger_dist(float): trigger distance to POI   
        trigger_hae(float): trigger height above ellipsoid
        trigger_agl(float): trigger height above ground
        """
    
        self.flightphase_FROM = flightphase_FROM
        self.flightphase_TO = flightphase_TO
        self.trigger_dist = trigger_dist
        self.trigger_hae = trigger_hae
        self.trigger_agl = trigger_agl

    def evaluate(self, dist: float, hae: float, agl: float, flightphase: FlightphaseEnum) -> FlightphaseEnum:
        """
        Evalutes the trigger and returns the new flight phase, if successful and 'None' if not 

        Parameters:
        dist(float): distance to POI   
        hae(float): height above ellipsoid
        agl(float): height above ground
        flightphase(FlightphaseEnum): the current flightphase

        Returns:
        The new flight phase, if successful and 'None' if not
        """

        if (
            flightphase == self.flightphase_FROM
            and dist < self.trigger_dist
            and hae < self.trigger_hae
            and agl < self.trigger_agl
        ):
            return self.flightphase_TO
        return None

class TimedTriggerCondition:

    def __init__(
        self, flightphase_FROM : FlightphaseEnum, flightphase_TO : FlightphaseEnum,
        dT:float
    ) -> None:
        """
        Initializes the TriggerClass.

        Parameters:
        flightphase_FROM(FlightphaseEnum): the flightphase we want to switch from
        flightphase_TO(FlightphaseEnum): the flightphase we want to switch to
        dT (float): time to flightphase change
        """
    
        self.flightphase_FROM = flightphase_FROM
        self.flightphase_TO = flightphase_TO
        self.trigger_dT = dT
        self.last_time = time.time()

    def evaluate(self, flightphase: FlightphaseEnum) -> FlightphaseEnum:
        """
        Evalutes the trigger and returns the new flight phase, if successful and 'None' if not 

        Parameters:
        flightphase(FlightphaseEnum): the current flightphase

        Returns:
        The new flight phase, if successful and 'None' if not
        """

        current_time = time.time()
        if flightphase == self.flightphase_FROM:
            if current_time-self.last_time >= self.trigger_dT:
                return self.flightphase_TO
            return None
        # keep up with time ellapsed if we do not need to check, i.e. if we are not in the TO-flightphase
        self.last_time = current_time
            
class FSM:
    def __init__(self, use_timed_trigger = False):
        """
        Initializes the Finite State Machine node.

        Parameters:
        use_timed_trigger(bool): decides if the trigger shall be mocked by a simple timer
        """
        self.use_timed_trigger = use_timed_trigger
        self.triggers = []
        # TODO make configurable
        if (not use_timed_trigger):
            self.triggers.append(
                TriggerCondition(
                    FlightphaseEnum.CRUISE, FlightphaseEnum.DESCENT,
                    trigger_dist=4000)
            )
            self.triggers.append(
                TriggerCondition(
                    FlightphaseEnum.DESCENT, FlightphaseEnum.APPROACH,
                    trigger_dist=2000, trigger_agl=150)
            )
            self.triggers.append(
                TriggerCondition(
                    FlightphaseEnum.APPROACH, FlightphaseEnum.FINAL_APPROACH,
                    trigger_dist=200, trigger_agl=30)
            )
            self.triggers.append(
                TriggerCondition(
                    FlightphaseEnum.FINAL_APPROACH, FlightphaseEnum.LANDING,
                    trigger_dist=4, trigger_agl=10)
            )
        else:
            self.triggers.append(
                TimedTriggerCondition(
                    FlightphaseEnum.CRUISE, FlightphaseEnum.DESCENT,
                    dT=2)
            )
            self.triggers.append(
                TimedTriggerCondition(
                    FlightphaseEnum.DESCENT, FlightphaseEnum.APPROACH,
                    dT=2)
            )
            self.triggers.append(
                TimedTriggerCondition(
                    FlightphaseEnum.APPROACH, FlightphaseEnum.FINAL_APPROACH,
                    dT=2)
            )
            self.triggers.append(
                TimedTriggerCondition(
                    FlightphaseEnum.FINAL_APPROACH, FlightphaseEnum.LANDING,
                    dT=2)
            )
            self.triggers.append(
                TimedTriggerCondition(
                    FlightphaseEnum.LANDING, FlightphaseEnum.CRUISE,
                    dT=2)
            ) 

        self.current_flightphase = FlightphaseEnum.CRUISE

    def evaluateTimed(self) -> None:
        """
        Evaluates the FSM based on timed triggers and sets the current flightphase
        """
        for trigger in self.triggers:
            nextphase = trigger.evaluate(self.current_flightphase)
            if nextphase is not None:
                self.current_flightphase = nextphase
    def evaluate(self, dist: float, hae: float, agl: float) -> None:
        """
        Evaluates the FSM and sets the current flightphase

        Parameters:
        dist(float): distance to POI   
        hae(float): height above ellipsoid
        agl(float): height above ground
        """
        for trigger in self.triggers:
            nextphase = trigger.evaluate(dist, hae, agl, self.current_flightphase)
            if nextphase is not None:
                self.current_flightphase = nextphase

class FlightphaseAssistent(Node):

    def __init__(self, use_timed_trigger = False, use_manual_mode=False) -> None:
        """
        Inites the FlightphaseAssistent class

        Parameters:
        use_timed_trigger(bool): decides if the trigger shall be mocked by a simple timer
        use_timed_trigger(bool): decides if the flightphase shall be set by a service call
        """
        super().__init__('flightphase_assistent')

        self.use_timed_trigger = use_timed_trigger
        self.use_manual_mode = use_manual_mode


        self.fsm = FSM(use_timed_trigger)

        self.latest_flightstate = Flightstate()
        self.current_flightphase = Flightphase()
        self.current_flightphase.header.frame_id = "/flight_phase"
        self.current_flightphase.flightphase = Flightphase.CRUISE
        
        self.flightphase_publisher = self.create_publisher(Flightphase, '/flightphase', 10)

        if (use_manual_mode):
            self.srv = self.create_service(SetFlightPhase, 'set_flightphase', self.manual_set_flightphase)

        self.timer_period = 0.2  # seconds
        self.flighphase_timer = self.create_timer(self.timer_period, self.timer_callback)

        if (not use_manual_mode and not use_timed_trigger):
            self.flightstate_subscription = self.create_subscription(
                Flightstate,
                '/flightstate',
                self.subscriber_callback,
                10)

        # TODO make configurable
        self.landing_spot_ned = (-2608.103, -3409.472)

        self.get_logger().info('Starting Flightphase Assistant')

    def subscriber_callback(self, msg: Flightstate) -> None:
        """
        Callback of the flightstate subscriber, stores the latest flightstate

        Parameters:
        msg(Flightstate): flightstate message
        """
        self.latest_flightstate = msg

    def manual_set_flightphase(self, request, response)->None:
        """
        Callback of the setFlightstate service, publishes flightphase, if existing

        Parameters:
        request, response: ros service params
        """

        print("got service call:", request.flightphase)

        if request.flightphase >=0 and request.flightphase<= 5:
            self.current_flightphase.flightphase = request.flightphase
            self.get_logger().info("manual flightphase set to " + flightphaseToString(FlightphaseEnum(self.current_flightphase.flightphase)))

            response.success = True
            return response

        self.get_logger().info("invalid flight phase requested")
        response.success = False
        return response

    def timer_callback(self) -> None:
        """
        Callback of the internal timer, processes the latest flightstate and calls the FSM to determine and publish the flightphase
        """
        if not self.use_manual_mode:
            if self.use_timed_trigger:
                self.fsm.evaluateTimed()
            else: 
                self.fsm.evaluate(
                    self.get_dist_2_ls(),
                    self.latest_flightstate.hae,
                    self.latest_flightstate.agl
                )
                
            if (self.current_flightphase.flightphase != self.fsm.current_flightphase.value):
                print( self.fsm.current_flightphase)
                self.get_logger().info('Set Flightphase to ' + flightphaseToString(self.fsm.current_flightphase) )

            self.current_flightphase.flightphase = self.fsm.current_flightphase.value

        self.current_flightphase.header.stamp = self.get_clock().now().to_msg()
        self.flightphase_publisher.publish(self.current_flightphase)

    def get_dist_2_ls(self) -> float:
        """
        Calculates the distance to the POI based on the latest flightstate

        Returns:
        Distance to the POI (float)
        """
        dist = math.sqrt(
            (self.latest_flightstate.ned_north - self.landing_spot_ned[0])**2 +
            (self.latest_flightstate.ned_east - self.landing_spot_ned[1])**2
        )
        return dist



def main():
    rclpy.init()

    if len(sys.argv) > 1 and sys.argv[1] == "flightstate":
        print("USING FLIGHTSTATE as TRIGGERS")
        use_timed_trigger = False
        use_manual_trigger = False
    if len(sys.argv) > 1 and sys.argv[1] == "timed":
        print("USING TIMED TRIGGERS")
        use_timed_trigger = True
        use_manual_trigger = False
    else:
        print("STARTING AS SERVICE")
        use_timed_trigger = False
        use_manual_trigger = True


    fpa = FlightphaseAssistent(use_timed_trigger, use_manual_trigger)
    rclpy.spin(fpa)


if __name__ == '__main__':
    main()
