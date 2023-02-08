from setuptools import setup

package_name = 'rest_to_rest'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/Q.npy'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bencic',
    maintainer_email='daniel.bencic@thi.de',
    description='Rest to Rest Trajectory Generation with Q Learning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rest_to_rest = rest_to_rest.rest_to_rest:main'
        ],
    },
)
