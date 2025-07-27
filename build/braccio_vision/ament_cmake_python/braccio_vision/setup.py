from setuptools import find_packages
from setuptools import setup

setup(
    name='braccio_vision',
    version='1.0.0',
    packages=find_packages(
        include=('braccio_vision', 'braccio_vision.*')),
)
