from setuptools import setup, find_packages
import os

setup(
    name='protocol_lib_up70',       
    version='1.0',                  
    description='A communication protocol library of NJUST-UP70-Vision',  
    long_description=open(os.path.join(os.path.dirname(__file__), 'README.md'), encoding='utf-8').read(),       
    long_description_content_type='text/markdown',   
    author='XiaomoWen',            
    author_email='1328821028@qq.com',          
    packages=find_packages(),       
    install_requires=[              
        'pyserial',
        'crcmod'
    ],
    python_requires='>=3.9',         
)
