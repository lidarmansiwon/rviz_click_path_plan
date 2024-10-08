from setuptools import setup
import os
from glob import glob

package_name = 'rviz_click_path_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['rviz/rcpp.rviz']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sw',
    maintainer_email='kimsanmaro@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_planner = rviz_click_path_plan.rviz_click_path_planner:main',
            'record_path_generator = rviz_click_path_plan.record_path_generator:main',
        ],
    },
)
