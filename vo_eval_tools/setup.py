from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'vo_eval_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mel',
    maintainer_email='mel5469singtel@gmail.com',
    description='Automated figure-8 flight + rosbag recording + ov_eval-based accuracy '
                'evaluation for OpenVINS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'figure8_controller = vo_eval_tools.figure8_controller:main',
            'bag_to_tum = vo_eval_tools.bag_to_tum:main',
            'run_evaluation = vo_eval_tools.run_evaluation:main',
            'plot_evaluation = vo_eval_tools.trajectory_plot:main',
        ],
    },
)
