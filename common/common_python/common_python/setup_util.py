import os
from typing import List, Tuple


def add_directory(package_name: str, target_directory: str, data_files: List[Tuple[str, List[str]]]) -> None:
    """Add `target_directory` to `data_files`

    Parameters:
    ----------
    `package_name`: Package name to build
    `target_directory`: Directory to copy to install
    `data_files`: A list of files to be copied to install by colcon build

    Examples:
    ----------
    See `get_data_files()` function
    """
    for (path, directories, filenames) in os.walk(target_directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            data_files.append(
                (os.path.join("share", package_name, path), [file_path]))


def get_data_files(package_name: str, target_directories: Tuple[str, ...] = ()) -> List[Tuple[str, List[str]]]:
    """Returns a list of files to be copied to install by colcon build
    Used in setup.py

    Parameters:
    ----------
    `package_name`: Package name to build
    `target_directories`: Directories to copy to install

    Returns:
    ----------
    a list of files to be copied to install

    Examples:
    ----------
    >>> data_files=get_data_files("sample_pkg"),
    copy 'resource/sample_pkg', 'package.xml'
    >>> data_files=get_data_files("sample_pkg", ("launch",)),
    copy 'resource/sample_pkg', 'package.xml', 'launch'
    >>> data_files=get_data_files("sample_pkg", ("launch", "urdf")),
    copy 'resource/sample_pkg', 'package.xml', 'launch', 'urdf'

    Notes
    -----
    When `target_directories` is one, it is not recognized as a tuple without a comma !
        NG: `get_data_files("sample_pkg", ("launch"))`
        OK: `get_data_files("sample_pkg", ("launch",))`
    """
    data_files = [
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join("share", package_name), ['package.xml']),
    ]
    for target_directory in target_directories:
        add_directory(package_name, target_directory, data_files)
    return data_files
