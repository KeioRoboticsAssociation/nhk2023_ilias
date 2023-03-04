from setuptools import setup
from glob import glob
import os

package_name = 'simple_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include img dir
        ('share/' + package_name +'/img', glob(os.path.join('simple_gui/img', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='curious',
    maintainer_email='curious.ks.jp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_gui = simple_gui.simple_gui:main',
        ],
    },
)
