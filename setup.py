from setuptools import setup

package_name = 'red_object_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Detect red objects and compute 3D positions',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_detector_node = red_object_detector.red_detector_node:main'
        ],
    },
)

