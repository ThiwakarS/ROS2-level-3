from setuptools import find_packages, setup

package_name = 'turtle_final_project'

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
    maintainer='dragon-booster',
    maintainer_email='dragon-booster@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtlesim_action_server=turtle_final_project.turtlesim_action_server:main",
            "turtlesim_node_manager=turtle_final_project.turtlesim_node_manager:main"
        ],
    },
)
