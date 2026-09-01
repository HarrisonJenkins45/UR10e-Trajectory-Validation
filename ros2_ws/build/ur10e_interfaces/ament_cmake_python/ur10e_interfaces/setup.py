from setuptools import find_packages
from setuptools import setup

setup(
    name='ur10e_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('ur10e_interfaces', 'ur10e_interfaces.*')),
)
