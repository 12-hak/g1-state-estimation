import os
from setuptools import setup
from glob import glob

package_name = 'g1_web_interface'

# Build data_files list preserving frontend dist directory structure
frontend_data_files = []
dist_dir = 'frontend/dist'
if os.path.isdir(dist_dir):
    for dirpath, dirnames, filenames in os.walk(dist_dir):
        if filenames:
            rel = os.path.relpath(dirpath, dist_dir)
            dest = os.path.join('share', package_name, 'frontend', rel).replace('\\', '/')
            files = [os.path.join(dirpath, f) for f in filenames]
            frontend_data_files.append((dest, files))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ] + frontend_data_files,
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='G1 Dev',
    maintainer_email='dev@unitree.local',
    description='Web interface for G1 SLAM and navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_bridge_node = g1_web_interface.web_bridge_node:main',
        ],
    },
)
