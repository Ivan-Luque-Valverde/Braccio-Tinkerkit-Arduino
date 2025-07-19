from setuptools import find_packages
from setuptools import setup

setup(
    name='braccio_bringup',
    version='0.0.0',
    packages=find_packages(
        include=('braccio_bringup', 'braccio_bringup.*')),
)
