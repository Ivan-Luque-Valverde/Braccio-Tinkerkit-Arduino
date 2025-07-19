from setuptools import find_packages
from setuptools import setup

setup(
    name='braccio_hardware',
    version='0.0.0',
    packages=find_packages(
        include=('braccio_hardware', 'braccio_hardware.*')),
)
