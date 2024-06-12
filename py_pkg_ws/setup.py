from setuptools import setup

package_name = 'py_pkg_ws'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdelaziz',
    maintainer_email='abdelazizmaher17499@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "number_publisher = py_pkg_ws.number_publisher:main",
            "number_counter = py_pkg_ws.number_counter:main"
        ],
    },
)
