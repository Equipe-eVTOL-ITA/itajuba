from setuptools import setup, find_packages
import os
from glob import glob
# setuptools é uma biblioteca python que permite a criação de pacotes

package_name = 'itajuba_cv_utils' # este nome deve corresponder ao nome no package.xml

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files (if they exist)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files (if they exist)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'], #, 'ultralytics'],
    zip_safe=True,
    maintainer='Angelo Marconi Pavan',
    maintainer_email='angelo.marconi.pavan@gmail.com',
    description='ROS 2 package for computer vision utilities.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={

        # Define pontos de entrada para o pacote.
        # Permite que o setuptools crie scripts executáveis que chamam funções específicas no seu código

        'console_scripts': [
            #'yolo_classifier = yolo_classifier.yolo_classifier:main',
            #'barcode = barcode_detector.oak_bar:main',
            'lane = lane_detector.lane_detector:main' # ros2 run itajuba_cv_utils lane
                    #  pasta       arquvivo.py  funcao no arquivo
        ],
    }
)
