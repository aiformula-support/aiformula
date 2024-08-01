from setuptools import setup
from common_python.setup_util import get_data_files

package_name = 'rear_potentiometer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=get_data_files(package_name, ("launch",)),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='issaomura',
    maintainer_email='issa_omura@jp.honda',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rear_potentiometer = rear_potentiometer.rear_potentiometer:main',
        ],
    },
)
