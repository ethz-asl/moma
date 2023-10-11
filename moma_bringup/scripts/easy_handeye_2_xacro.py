import re
import os
import argparse

import scipy as sp
import numpy as np
import yaml
from pathlib import Path

"""
Input: xacro file & easy handeye result into .txt or .yaml file
File converts quaternions to euler transforms, and puts results in xacro file

To replace manual copy/paste procedure.

1. cp panda_arm.xacro to this location.
2. create easy_handeye_file.txt with the easy handeye result after calibration
"""


class CommandLineArgs:
    """
    Take arguments from command line
    Parameters:
    ---------------------------------------------------------
    easy_handeye_file (-e) : text file or yaml file from easy handeye result
    xacro_file (-x) : xacro file for arm that needs updating
    """

    def __init__(self):
        parser = argparse.ArgumentParser(
            description="Less manual calibration",
            fromfile_prefix_chars="@",
            allow_abbrev=False,
        )
        parser.add_argument(
            "-e",
            "--easy_handeye_file",
            action="store",
            type=self._file_path,
            help="set file name of txt file from easy handeye calibration result",
        )

        parser.add_argument(
            "-x",
            "--xacro_file",
            action="store",
            type=self._file_path,
            help="set file name of xacro file to change",
        )

        self.args = parser.parse_args()
        print("Command Line Arguments: ", vars(self.args))

    def _file_path(self, string):
        if os.path.isfile(string):
            return string
        else:
            print("-------------------------------------------")
            print("String: %s is not a file" % (string))
            print("-------------------------------------------")
            print("Please input file paths correctly")
            print("-------------------------------------------")
            raise FileNotFoundError


def from_easy_handeye(easy_handeye_file):
    """
    Opens file of result from easy handeye, and converts to dict.
    in future prehaps take input from easy_handeye rviz GUI.
    
    File should look like this:

    translation:
      x: 0.01185524852091389
      y: -0.02569462330148611
      z: 0.07128557290494696
    rotation:
      x: 0.6468698017615132
      y: -0.2542335707423363
      z: 0.6650909062427098
      w: 0.27309126223640573
    """
    try:
        with open(Path(easy_handeye_file)) as _file:
            try:
                conf = yaml.safe_load(_file)
            except yaml.YAMLError as err:
                print(err)

        return conf

    except FileNotFoundError:
        print(f"Error: file {easy_handeye_file} not found")
    except Exception as err:
        print(f"an error occured: {str(err)}")


def convert_quat_euler(quat, typ="xyz"):
    """
    Converts from quaternion to Euler representation.
    
    quat (list) : 4 x 1 list with [x, y, z, w] format
    typ (str) : type of Euler rotation

    ZYX -> intrinsic
    xyz -> extrinsic equivalent
    
    returns (list) : Euler representation [x, y, z]
    """
    r = sp.spatial.transform.Rotation
    return r.from_quat(quat).as_euler(typ, degrees=False)


def replace_wrist_calibration(file_name, translation, rotation):
    """
    regex patterns to search for and replace inside the xacro file
    finds wrist calibration and replaces everything inside ' '
    """
    patterns = [
        (
            r"wrist_calibration_rpy:=\'(.*?)\'",
            f"wrist_calibration_rpy:='{rotation[0]} {rotation[1]} {rotation[2]}'",
        ),
        (
            r"wrist_calibration_xyz:=\'(.*?)\'",
            f"wrist_calibration_xyz:='{translation[0]} {translation[1]} {translation[2]}'",
        ),
    ]

    try:
        # read xacro file
        with open(file_name, "r") as xacro_file:
            xacro_content = xacro_file.read()

        for pattern, replacement in patterns:
            xacro_content = re.sub(pattern, replacement, xacro_content)

        # write new translations to xacro file
        with open(file_name, "w") as xacro_file:
            xacro_file.write(xacro_content)

        print(f"Sucessfully updated {file_name}")

    except FileNotFoundError:
        print(f"Error: file {file_name} not found")
    except Exception as err:
        print(f"an error occured: {str(err)}")


def main():

    cli = CommandLineArgs()
    if cli.args.easy_handeye_file is None:
        easy_handeye_file = "easy_handeye_file.txt"
    else:
        easy_handeye_file = cli.args.easy_handeye_file

    if cli.args.xacro_file is None:
        xacro_file = "panda_arm.xacro"
    else:
        xacro_file = cli.args.xacro_file

    result = from_easy_handeye(easy_handeye_file)

    translation = [val for val in result["translation"].values()]

    quat = [val for val in result["rotation"].values()]
    rotation = convert_quat_euler(quat)

    print(
        "Translation x: %.3f y: %.3f z: %.3f"
        % (translation[0], translation[1], translation[2])
    )
    print(
        "Quaternion x: %.3f y: %.3f z: %.3f, w: %.3f"
        % (quat[0], quat[1], quat[2], quat[3])
    )
    print(
        "Euler Rotation x: %.3f y: %.3f z: %.3f"
        % (rotation[0], rotation[1], rotation[2])
    )

    confirm = (
        input(
            f"Are you sure you want to edit the wrist calibration in {xacro_file}? (y/n) "
        )
        .strip()
        .lower()
    )

    if confirm != "y":
        print("Not editing file.")
        return
    else:
        replace_wrist_calibration(xacro_file, translation, rotation)


if __name__ == "__main__":
    main()
