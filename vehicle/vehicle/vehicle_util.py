import os.path as osp
import xacro
from ament_index_python.packages import get_package_share_directory


def get_zed_intrinsic_param_path(serial_number: str, resolution: str) -> str:
    """Returns the zed intrinsic yaml path for the given `serial_number`

    Parameters:
    ----------
    `serial_number`: Zed Serial Number
    `resolution`

    Returns:
    ----------
    the intrinsic yaml path for given `serial_number`

    Examples:
    ----------
    >>> get_zed_intrinsic_param_path("SN48442725", "HD1080")
    /path/to/colcon_ws/install/vehicle/share/vehicle/config/zedx/camera_params/SN48442725/HD1080.yaml\
    """
    intrinsic_yaml_path = osp.join(get_package_share_directory("vehicle"),
                                   "config/zedx/camera_params/", serial_number, "intrinsic_" + resolution + ".yaml")
    if not osp.exists(intrinsic_yaml_path):
        raise FileNotFoundError(intrinsic_yaml_path)
    return intrinsic_yaml_path


def convert_xacro_to_urdf(xacro_path: str, urdf_path: str) -> None:
    """Convert xacro file to urdf file

    Parameters:
    ----------
    `xacro_path`: Source xacro file path
    `urdf_path`: Converted urdf file path

    Examples:
    ----------
    >>> convert_xacro_to_urdf("/path/to/robot.xacro", "/path/to/robot.urdf")
    """
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent="  ")
    f = open(urdf_path, "w")
    f.write(robot_desc)
    f.close()
