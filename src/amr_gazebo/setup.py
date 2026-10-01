import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_gazebo'

def get_model_data_files(package_name, base_dir='models'):
    data_files = []
    for root, dirs, files in os.walk(base_dir):
        if files:
            dest = os.path.join('share', package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((dest, file_paths))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ] + get_model_data_files(package_name),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quangtran',
    maintainer_email='trandanhquang2005@gmail.com',
    description='Gazebo simulation worlds and launch files for Food Delivery AMR',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
