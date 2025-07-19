from setuptools import find_packages
from setuptools import setup

setup(
    name='braccio_description',
    version='0.0.0',
    packages=find_packages(
        include=('braccio_description', 'braccio_description.*')),
)
