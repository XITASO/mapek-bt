import random
import yaml
import os
import sys


def generate_random_stamp(t_min, t_max):
    # Generate a random float between 5.0 and 25.0
    return round(random.uniform(t_min, t_max), 1)


def create_yaml_file(base_path:str, start_time:float, intervall:float):
    # new timestamp for message drop
    data = [
        {
            "type": "parameter_adaptation",
            "target_node": "/managed_subsystem/camera",
            "name": "do_drop_rgb_camera",
            "value": True,
            "stamp": generate_random_stamp(start_time, start_time+intervall),
            "is_gt_failure": True,
        }
    ]
    with open(os.path.join(base_path, "scenario_message_drop.yaml"), "w") as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)

    # new timestamp for image_degradation
    data = [
        {
            "type": "parameter_adaptation",
            "target_node": "/managed_subsystem/camera",
            "name": "image_degradation",
            "value": 1.0,
            "stamp": generate_random_stamp(start_time, start_time+intervall),
            "is_gt_failure": True,
        }
    ]
    with open(
        os.path.join(base_path, "scenario_image_degradation.yaml"), "w"
    ) as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)
    
    # new timestamp for F1+S1
    data = [
        {
            "type": "parameter_adaptation",
            "target_node": "/managed_subsystem/camera",
            "name": "image_degradation",
            "value": 1.0,
            "stamp": generate_random_stamp(start_time, start_time+intervall),
            "is_gt_failure": True,
        },
        {
            "type": "change_flightphase",
            "target_service": "/managing_subsystem/set_flightphase",
            "value": 2,
            "stamp": generate_random_stamp(start_time, start_time+intervall),
            "is_gt_failure": False,
        },
    ]
    with open(
        os.path.join(base_path, "scenario_F1+S1.yaml"), "w"
    ) as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)


if __name__ == "__main__":
    base_path = "src/experiment_setup/resources/scenarios"
    start = float(sys.argv[1])
    # 5 senconds of start up, then 5 seconds no fail, 20 sec potential trigger, 5 sec end
    create_yaml_file(base_path=base_path, start_time=start+10, intervall=20)
