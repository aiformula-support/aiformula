from setuptools import find_packages, setup
from common_python.setup_util import get_data_files

package_name = 'sample_vehicle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=get_data_files(package_name, ("config", "launch", "xacro", "rviz")),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuki Akimoto',
    maintainer_email='yuki_akimoto@jp.honda',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
