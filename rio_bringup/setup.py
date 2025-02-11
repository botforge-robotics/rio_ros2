from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rio_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools',
                    'aiohttp',
                    'aiohttp_cors',
                    'aiortc',
                    'opencv-python',
                    'ollama'],
    zip_safe=True,
    maintainer='chaitu',
    maintainer_email='nagachaitanya948@gmail.com',
    description='Rio bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = rio_bringup.odom_tf_broadcaster:main',
            'lidar_udp_node = rio_bringup.lidar_udp_node:main',
            'webrtc_node = rio_bringup.webrtc_node:main',   
            'ollama_nlp_node = rio_bringup.ollama_nlp_node:main',
        ],
    },
)
