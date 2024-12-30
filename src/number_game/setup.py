from setuptools import setup

package_name = 'number_game'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Number baseball game with reinforcement learning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'random_number_publisher = number_game.random_number_publisher:main',
        'number_guessing_agent = number_game.number_guessing_agent:main',
        'answer_checker = number_game.answer_checker:main',  # 새 노드 추가
    ],
}
)

