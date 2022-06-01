from setuptools import setup

package_name = 'worker_node'

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
    maintainer='meike',
    maintainer_email='meike.bloecher@fau.de',
    description='This package navigates the robot around the course accoring to the recieved messages in the mqtt connection node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'worker = worker_node.worker:main',
        ],
    },
)
