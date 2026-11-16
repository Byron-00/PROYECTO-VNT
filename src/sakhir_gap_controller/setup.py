
from setuptools import find_packages, setup

package_name = 'sakhir_gap_controller'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(
        exclude=['test']
    ),

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
        'numpy'
    ],

    zip_safe=True,

    maintainer='Byron Ortega',

    description=(
        'Reactive Follow the Gap controller '
        'for F1TENTH autonomous racing on Sakhir.'
    ),

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'gap_controller = '
            'sakhir_gap_controller.gap_controller:main',
        ],
    },
)
