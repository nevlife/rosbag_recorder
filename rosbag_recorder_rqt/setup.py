from setuptools import setup

package_name = 'rosbag_recorder_rqt'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nevlife',
    maintainer_email='nevlife000720@gmail.com',
    description='rqt plugin for rosbag_recorder.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbag_recorder_rqt = rosbag_recorder_rqt.recorder:main',
        ],
    },
)
