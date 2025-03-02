from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'llm_action'
# Use glob to include all files from the 'templates' directory
prompts_files = glob('templates/*')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates') , prompts_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='oleg.grinets@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_action_node = llm_action.llm_action_server:main',
            'llm_light_action_node = llm_action.llm_action_server_light_llm:main',
            'llm_vision_action_node = llm_action.llm_action_server_vision_llm:main',
            'llm_action_client = llm_action.llm_action_client:main',
            'llm_intent_ident_node = llm_action.llm_intent_ident_node:main',
            'llm_vision_processor_node = llm_action.llm_vision_processor_node:main',
        ],
    },
)
