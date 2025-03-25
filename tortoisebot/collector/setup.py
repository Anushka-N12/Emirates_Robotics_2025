from setuptools import find_packages, setup

package_name = 'collector'

# Function to read the requirements.txt file
def parse_requirements(filename):
    with open(filename, 'r') as file:
        return [line.strip() for line in file if line.strip() and not line.startswith('#')]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/yolo11n.pt']),  # Add the model file
    ],
    install_requires=parse_requirements('requirements.txt'),
    zip_safe=True,
    maintainer='anushka',
    maintainer_email='anushka@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colorDetectorPublisher = collector.color_detector_publisher_node:main',
            'colorDetectorSub = collector.color_detector_sub_node:main',
            'finalDump = collector.final_node:main'
        ],
    },
)