from setuptools import find_packages, setup

package_name = 'dsr_gss'

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
    maintainer='steve',
    maintainer_email='hermes7308@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tamping = dsr_gss.tamping:main',
            'tamping_client = dsr_gss.tamping_client:main',
            'test_server = dsr_gss.test_server:main',
            'tampingCopy = dsr_gss.tampingCopy:main',
            'tamping_with_gripping = dsr_gss.tamping_with_gripping:main'
        ],
    },
)
