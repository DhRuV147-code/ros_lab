from setuptools import find_packages, setup

package_name = 'test_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/remap.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'circle_timer_exe=test_turtle.circle_timer:main',
        'circle_exe=test_turtle.circle:main',
        'spiral_exe=test_turtle.spiral:main',
        'teleport_exe=test_turtle.teleport:main',
        'cycle_exe=test_turtle.cycle:main',
        'pub_sub=test_turtle.pub_sub:main'
        ],
    },
)
