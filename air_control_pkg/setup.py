# import os
# from glob import glob
from setuptools import setup
import glob
import os

package_name = 'air_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py')))
        # (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jgj2017',
    maintainer_email='jgj2017@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pm_server = air_control_pkg.particulate_matter_server:main',
            'pm_sensor = air_control_pkg.particulate_matter_sensor:main',
        	'control_unit = air_control_pkg.central_control_unit:main',
            'air_cleaner = air_control_pkg.air_cleaner:main',
        ],
    },
)
