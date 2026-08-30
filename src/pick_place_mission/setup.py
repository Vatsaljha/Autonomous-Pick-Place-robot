from setuptools import setup

package_name = 'pick_place_mission'

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

    install_requires=[
        'setuptools',
        'PyYAML',
    ],

    zip_safe=True,

    maintainer='vatsal',
    maintainer_email='vatsal@example.com',

    description=(
        'Autonomous TurtleBot3 '
        'pick and place mission'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'mission_manager = '
            'pick_place_mission.mission_manager:main',
        ],
    },
)