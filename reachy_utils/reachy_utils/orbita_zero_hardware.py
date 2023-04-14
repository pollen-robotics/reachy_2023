"""Read and store orbita zero in config file."""

from contextlib import closing
import os
import shutil
import struct
from typing import Dict, Tuple
import yaml

from pypot.dynamixel import DxlIO
from pypot.dynamixel.protocol.v1 import DxlReadDataPacket


PRESENT_POSITION_ADDR = 71


def get_orbita_current_position(
    serial_port: str, id: int, reduction: float
) -> Tuple[float, float, float]:
    """Connect and read the current orbita disks position."""
    print(f'Connecting on "{serial_port}" (id={id}, reduction={reduction})')
    with closing(DxlIO(serial_port)) as dxl_io:
        read_disk_pos_packet = DxlReadDataPacket(
            id, PRESENT_POSITION_ADDR, struct.calcsize("fff")
        )
        resp = dxl_io._send_packet(read_disk_pos_packet)
        disks = struct.unpack("fff", bytearray(resp.parameters))
        disks = [d / reduction for d in disks]

        pos = {"top": disks[0], "middle": disks[1], "bottom": disks[2]}
        print(f"Found current pos: {pos}")
        return pos


def update_config_with_zero(config_file: str, orbita_name: str, zero: Dict[str, float]):
    """Update config file with orbita zero."""
    config_file = os.path.expanduser(config_file)
    if os.path.exists(config_file):
        with open(config_file) as f:
            print(f'Found existing config in "{config_file}"...')

            backup_file = f"{config_file}.bkp"
            print(f'Making a backup in "{backup_file}"...')
            shutil.copy(config_file, backup_file)

            config = yaml.load(f, Loader=yaml.Loader)
    else:
        print("No config found, create one.")
        config = {}

    print(f'Updating config with zero for orbita "{orbita_name}"...')
    config[f"{orbita_name}_orbita_zero"] = {}
    config[f"{orbita_name}_orbita_zero"]["top"] = zero["top"]
    config[f"{orbita_name}_orbita_zero"]["middle"] = zero["middle"]
    config[f"{orbita_name}_orbita_zero"]["bottom"] = zero["bottom"]

    with open(config_file, "w") as f:
        yaml.dump(config, f, sort_keys=False)

    print("Done!")


def main():
    """Get orbita zero and store it in config file."""
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--config-file", default="~/.reachy.yaml")
    parser.add_argument("--orbita-name", default="neck")
    parser.add_argument("--serial_port", default="/dev/orbita_neck")
    parser.add_argument("--id", type=int, default=70)
    parser.add_argument("--reduction", type=float, default=64 / 15)
    args = parser.parse_args()

    disk_pos = get_orbita_current_position(args.serial_port, args.id, args.reduction)

    update_config_with_zero(args.config_file, args.orbita_name, disk_pos)


if __name__ == "__main__":
    main()
