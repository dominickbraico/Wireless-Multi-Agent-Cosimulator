from glob import glob

from setuptools import find_packages, setup

package_name = 'consensus_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominick',
    maintainer_email='dominickbraico32@gmail.com',
    description='Template controller: altitude consensus over the emulated radio',
    license='Apache-2.0',
    entry_points={'console_scripts': ['consensus_node = consensus_controller.consensus_node:main']},
)
