from setuptools import find_packages, setup

from common_python.setup_util import get_data_files

package_name = "extremum_seeking_mpc"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=get_data_files(package_name, ("config", "launch")),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Issa Omura",
    maintainer_email="issa_omura@jp.honda",
    description="Model Predictive Control using Extremum Seeking Controller",
    license="LGPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "extremum_seeking_mpc = extremum_seeking_mpc.extremum_seeking_mpc:main",
        ],
    },
)
