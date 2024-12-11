from setuptools import find_packages, setup

package_name = 'uwb'

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
        	'UWBReceiver = uwb.UWBReceiver:main',
        	'UWBDataAggregator = uwb.UWBDataAggregator:main',
        	'UWBDataAggregator2 = uwb.UWBDataAggregator2:main',
        	'UWBCalculator = uwb.UWBCalculator:main',
        	'UWBCalculator2 = uwb.UWBCalculator2:main',
        	'UWBPlotter = uwb.UWBPlotter:main',
        ],
    },
)
