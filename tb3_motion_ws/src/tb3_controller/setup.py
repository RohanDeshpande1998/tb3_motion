from setuptools import find_packages, setup

package_name = 'tb3_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['rviz/model.rviz']),
        ('share/' + package_name + '/launch', ['launch/tb3_simulation.launch.py', 'launch/tb3_rviz.launch.py']),
        ('share/' + package_name + '/config',['config/waypoints.yaml'])
        # Add data files if needed
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'matplotlib',
        'scipy',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Pure Pursuit and trajectory generator for TurtleBot3',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = tb3_controller.pure_pursuit_node:main',
            'trajectory_generator = tb3_controller.trajectory_generator:main',
            'plotter_node = tb3_controller.plotter_node:main',
        ],
    },
)

