from setuptools import find_packages
from setuptools import setup

setup(
    name='ml25d_dataset_generation',
    version='0.2.0',
    packages=find_packages(
        include=('ml25d_dataset_generation', 'ml25d_dataset_generation.*')),
)
