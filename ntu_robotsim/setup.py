from setuptools import setup

package_name = 'ntu_robotsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=['pkg'],
    package_dir={'pkg': 'pkg'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ntu-user',
    maintainer_email='ntu-user@todo.todo',
    description='NTU Robot Simulation',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_odometry = pkg.visual_odometry:main',
            'landmark_database = pkg.landmark_database:main',
        ],
    },
)
