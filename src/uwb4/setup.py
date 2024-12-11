from setuptools import find_packages, setup

package_name = 'uwb4'

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
    maintainer='li',
    maintainer_email='zcl.iarc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'UWBReceiver = uwb4.UWBReceiver:main',
        	'UWBDataAggregator = uwb4.UWBDataAggregator:main',
        	'UWBCalculator = uwb4.UWBCalculator:main',
        	'UWBPlotter = uwb4.UWBPlotter:main',
        	'UWBCalculator2 = uwb4.UWBCalculator2:main',
        	'UWBPlotter2 = uwb4.UWBPlotter2:main',
        ],
    },
)
