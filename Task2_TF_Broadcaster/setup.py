from setuptools import find_packages, setup

package_name = 'task2_tf_broadcaster'

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
    maintainer='ajith2204',
    maintainer_email='ajith2204@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'tf_broadcaster_node = task2_tf_broadcaster.tf_broadcaster_node:main',
        ],
    },
)
