from glob import glob
from setuptools import setup

package_name = 'bouncing_ball_catcher'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nama Kamu',
    maintainer_email='you@example.com',
    description='Game Bouncing Ball Catcher dengan ROS2 dan OpenCV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = bouncing_ball_catcher.camera_node:main',
            'game_node = bouncing_ball_catcher.game_node:main',
            'hsv_calibrator = bouncing_ball_catcher.hsv_calibrator:main',
        ],
    },
)
