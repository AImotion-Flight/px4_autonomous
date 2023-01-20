from setuptools import setup

package_name = 'rl_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/rl_planner/models', ['models/final_Q.npy'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bencic',
    maintainer_email='daniel.bencic@thi.de',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_planner = rl_planner.rl_planner:main'
        ],
    },
)
