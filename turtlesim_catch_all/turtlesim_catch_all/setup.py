from setuptools import find_packages, setup

package_name = 'turtlesim_catch_all'

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
    maintainer='ha',
    maintainer_email='khanhhado1208u@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller_targetpoint = turtlesim_catch_all.turtle_controller_targetpoint:main',
            'turtle_controller_catch_points = turtlesim_catch_all.turtle_controller_catch_points:main',
        ],
    },
)
