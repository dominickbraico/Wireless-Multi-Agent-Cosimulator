from setuptools import find_packages, setup

package_name = 'mesh_control'

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
    description='Mesh-dropout scenario: heartbeat node and random-flight node',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mesh_node = mesh_control.mesh_node:main',
        'mesh_flight = mesh_control.mesh_flight:main',
    ]},
)
