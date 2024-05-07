from setuptools import setup
from common_python.setup_util import get_data_files

package_name = 'common_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=get_data_files(package_name),
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
