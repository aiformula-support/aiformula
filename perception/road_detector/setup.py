from setuptools import setup
from common_python.setup_util import get_data_files

package_name = 'road_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=get_data_files(package_name, ("launch","weights","config")),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masayaokada',
    maintainer_email='masayaokada@jp.honda',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_detector=road_detector.road_detector:main'
        ],
    },
)
