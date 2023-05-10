import os
from serial import SerialException
from subprocess import run, PIPE
from typing import Dict
import yaml

from pypot.dynamixel import DxlIO, Dxl320IO
from reachy_utils.config import get_reachy_model

_latest_discovery_file = os.path.expanduser("~/.reachy-latest-discovery.yaml")


motor_ids_per_part = {
    "right_arm": [10, 11, 12, 13, 14, 15, 16, 17, 40, 41],
    "left_arm": [20, 21, 22, 23, 24, 25, 26, 27, 50, 51],
    "head": [30, 31, 60],
    "orbita_neck": [70],
}

robot_config_to_parts = {
    "full_kit": ["right_arm", "head", "left_arm"],
    "starter_kit_left": ["left_arm", "head"],
    "starter_kit_right": ["right_arm", "head"],
    "headless": ["right_arm", "left_arm"],
    "mini": ["head"],
    "starter_kit_right_no_head": ["right_arm", "orbita"]
}

motor_id_to_name = {
    10: "r_shoulder_pitch",
    11: "r_shoulder_roll",
    12: "r_arm_yaw",
    13: "r_elbow_pitch",
    14: "r_forearm_yaw",
    15: "r_wrist_pitch",
    16: "r_wrist_yaw",
    17: "r_gripper",
    20: "l_shoulder_pitch",
    21: "l_shoulder_roll",
    22: "l_arm_yaw",
    23: "l_elbow_pitch",
    24: "l_forearm_yaw",
    25: "l_wrist_pitch",
    26: "l_wrist_yaw",
    27: "l_gripper",
    30: "r_antenna",
    31: "l_antenna",
    40: "r_gripper_force_sensor",
    41: "r_fans",
    50: "l_gripper_force_sensor",
    51: "l_fans",
    60: "head_fans",
    70: "orbita_neck",
}


def get_missing_motors_arm(arm: str, missing_motors: Dict):
    try:
        dxl_io = DxlIO(port=f"/dev/usb2ax_{arm}")
    except SerialException:
        print(
            f"Port /dev/usb2ax_{arm} not found. Make sure that the udev rules is set and the usb2ax board plugged."
        )
        missing = [motor_id_to_name[motor_id] for motor_id in motor_ids_per_part[arm]]
        missing_motors[arm] = missing
        return missing_motors

    scan = dxl_io.scan(range(55))
    dxl_io.close()

    missing = [motor_id_to_name[motor_id] for motor_id in motor_ids_per_part[arm] if motor_id not in scan]
    missing_motors[arm] = missing

    return missing_motors


def get_missing_motors_head(missing_motors: Dict):
    try:
        dxl320_io = Dxl320IO(port="/dev/usb2ax_head")
    except SerialException:
        print(
            "Port /dev/usb2ax_head not found. Make sure that the udev rules is set and the usb2ax board plugged."
        )
        missing_motors["head"] = [motor_id_to_name[motor_id] for motor_id in motor_ids_per_part["head"]]
        return missing_motors

    scan = dxl320_io.scan([30, 31])
    dxl320_io.close()

    try:
        dxl_io = DxlIO(port='/dev/usb2ax_head')
    except SerialException:
        print(
            "Port /dev/usb2ax_head not found. Make sure that the udev rules is set and the usb2ax board plugged."
        )
        missing_motors["head"] = [motor_id_to_name[motor_id] for motor_id in motor_ids_per_part["head"]]
        return missing_motors

    scan += dxl_io.scan([60])
    dxl_io.close()

    missing_motors["head"] = [motor_id_to_name[motor_id] for motor_id in motor_ids_per_part["head"] if motor_id not in scan]

    return missing_motors


def check_if_orbita_missing(missing_motors: Dict):
    try:
        dxl_io = DxlIO(port="/dev/orbita_neck")
    except SerialException:
        print(
            "Port /dev/orbita_neck not found. Make sure that the udev rules is set and orbita plugged."
        )
        missing_motors["head"] += ["orbita_neck"]
        return missing_motors

    scan = dxl_io.scan(motor_ids_per_part['orbita_neck'])
    dxl_io.close()

    if scan == []:
        missing_motors["head"] += ["orbita_neck"]

    return missing_motors


def _init_missing_motors():
    missing_motors_init = {}

    for part in robot_config_to_parts[get_reachy_model()]:
        missing_motors_init[part] = []
    return missing_motors_init


def get_missing_motors_reachy(check_service: bool = True):
    reachy_model = get_reachy_model()
    missing_motors = _init_missing_motors()
    service_was_active = False

    pipe = run(
        ["systemctl --user is-active reachy_sdk_server.service"],
        stdout=PIPE,
        shell=True,
    )
    status = pipe.stdout.decode().split()

    if check_service:
        if status[0] == "active":
            service_was_active = True
            print("Disabling reachy_sdk_server.service to access the usb2ax boards.")
            run(
                ["systemctl --user stop reachy_sdk_server.service"], stdout=PIPE, shell=True
            )

    for part in robot_config_to_parts[reachy_model]:
        if "arm" in part:
            missing_motors = get_missing_motors_arm(part, missing_motors)

        elif "head" in part:
            missing_motors = get_missing_motors_head(missing_motors)
            missing_motors = check_if_orbita_missing(missing_motors)

        elif "orbita" in part:
            missing_motors = check_if_orbita_missing(missing_motors)

    if service_was_active:
        run(
                ["systemctl --user start reachy_sdk_server.service"], stdout=PIPE, shell=True
        )

    with open(_latest_discovery_file, 'w') as f:
        yaml.dump(missing_motors, f)

    return missing_motors


def scan():
    reachy_model = get_reachy_model()
    print(f"Scanning if there are any missing motors for Reachy {reachy_model}...")

    missing_motors = get_missing_motors_reachy()

    if missing_motors == _init_missing_motors():
        print(f"Found all motors for Reachy {reachy_model}!")
    else:
        print(f"Found missing motors for Reachy {reachy_model}: {missing_motors}")


if __name__ == "__main__":
    scan()
