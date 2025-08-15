from setuptools import setup, find_packages

package_name = 'steerai_demo_lifecycle_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='puneettu664@gmail.com',
    description='Lifecycle-based turtlesim controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'node = steerai_demo_lifecycle_controller.node:main',
        ],
    },
)
