from setuptools import find_packages, setup

package_name = 'swarm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominick',
    maintainer_email='dominickbraico32@gmail.com',
    description='Leader-follower swarm logic',
    license='Apache-2.0',
    entry_points={'console_scripts': ['swarm_node = swarm_control.swarm_node:main']},
)
