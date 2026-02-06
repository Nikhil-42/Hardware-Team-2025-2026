from setuptools import find_packages, setup

package_name = 'py_hub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil Iyer',
    maintainer_email='iyer.nikhil@ufl.com',
    description='Contains nodes related to the interfacing with hardware from the Raspberry Pi 5. (but in Python)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'service = py_hub.finger:main',
            'client = py_hub.finger_client:main',
        ],
    },
)
