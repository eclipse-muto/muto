from setuptools import find_packages, setup

package_name = 'muto_talker'

setup(
    name=package_name,
    version='0.42.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='composiv.ai',
    maintainer_email='info@composiv.ai',
    description='Eclipse Muto sample talker node that publishes string messages to demonstrate ROS 2 pub-sub communication',
    license='Eclipse Public License v2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'muto_talker = muto_talker.muto_talker:main'
        ],
    },
)
