from setuptools import setup

package_name = 'generic_ui'

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
    maintainer='cole',
    maintainer_email='github@coletek.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gcontroller = generic_ui.gcontroller:main',
            'gcam = generic_ui.gcam:main',
            'gcambox = generic_ui.gcambox:main'
        ],
    },
)
