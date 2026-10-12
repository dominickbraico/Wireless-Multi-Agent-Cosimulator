from setuptools import find_packages, setup

package_name = 'px4_offboard_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominick',
    maintainer_email='dominickbraico32@gmail.com',
    description='Minimal ROS 2 offboard control example for PX4 SITL bringups.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_control = px4_offboard_control.offboard_control:main',
            'square_trajectory = px4_offboard_control.square_trajectory:main',
            'survey_trajectory = px4_offboard_control.survey_trajectory:main',
            'circle_trajectory = px4_offboard_control.circle_trajectory:main',
            'follower_flight = px4_offboard_control.follower_flight:main',
        ],
    },
)
