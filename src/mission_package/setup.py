from setuptools import setup, find_packages

setup(
    name='mission_package',
    version='0.0.0',
    packages=find_packages('scripts'),
    package_dir={'': 'scripts'},
    install_requires=['rospy', 'std_msgs', 'interface_package'],
    zip_safe=False,
)
