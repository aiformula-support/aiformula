"""Convert xacro file to urdf file

Parameters:
----------
`xacro_path`: Source xacro file path
`urdf_path`: Converted urdf file path

Examples:
----------
>>> python3 convert_xacro_to_urdf.py /path/to/robot.xacro /path/to/robot.urdf
"""

from argparse import ArgumentParser
from vehicle.vehicle_util import convert_xacro_to_urdf

if __name__ == "__main__":

    parser = ArgumentParser(description="Xacro to URDF Converter")
    parser.add_argument("xacro_path", type=str,
                        help="Path to the input Xacro file")
    parser.add_argument("urdf_path", type=str,
                        help="Path to the output URDF file")
    args = parser.parse_args()
    convert_xacro_to_urdf(args.xacro_path, args.urdf_path)
