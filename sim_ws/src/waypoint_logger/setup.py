from setuptools import setup

package_name = 'waypoint_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='judson35',
    maintainer_email='79773875+judson35@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_logger=waypoint_logger.waypoint_logger:main',
            'waypoint_publisher=waypoint_logger.waypoint_visualizer:main'
        ],
    },
)
