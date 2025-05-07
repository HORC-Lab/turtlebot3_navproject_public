from setuptools import find_packages, setup

package_name = 'delta_t_tools'

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
    maintainer='apollo',
    maintainer_email='johncahill4493@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'relay_node = delta_t_tools.relay_node:main',
            'logger_node = delta_t_tools.logger_node:main',
        ],
    },
)
