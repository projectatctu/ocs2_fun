from setuptools import setup, find_packages

setup(
    name='ocs2_api project',
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
)