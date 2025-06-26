from setuptools import find_packages
from setuptools import setup

setup(
    name='traffic_information_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('traffic_information_msgs', 'traffic_information_msgs.*')),
)
