from setuptools import setup

package_name = 'custom_aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vatsal',
    maintainer_email='vatsal@example.com',
    description='Custom ArUco detector for TurtleBot3 Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = custom_aruco_detector.aruco_detector:main',
        ],
    },
)
