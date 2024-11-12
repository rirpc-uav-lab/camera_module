from setuptools import find_packages, setup

package_name = 'camera_module'

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
    maintainer='oleg',
    maintainer_email='sqrt2022@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        f'micasense_commands = {package_name}.micasense_commands:main',
        f'test_client = {package_name}.test_client:main',
        ],
    },
)
