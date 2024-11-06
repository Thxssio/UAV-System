from setuptools import setup, find_packages

setup(
    name='interface_package',
    version='0.0.0',
    packages=find_packages('scripts'),
    package_dir={'': 'scripts'},
    install_requires=['rospy', 'std_msgs', 'actionlib', 'actionlib_msgs'],
    zip_safe=False,
)
