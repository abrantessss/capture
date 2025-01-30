from setuptools import find_packages, setup

package_name = 'capture_claw'

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
    maintainer='Luís Abrantes',
    maintainer_email='luis.g.abrantes@tecnico.ulisboa.pt',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_claw_py = capture_claw.control_claw:main'
        ],
    },
)
