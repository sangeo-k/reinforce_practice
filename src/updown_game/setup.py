from setuptools import setup

package_name = 'updown_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Up and Down Game with Reinforcement Learning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_provider = updown_game.number_provider:main',
            'number_comparator = updown_game.number_comparator:main',
            'agent = updown_game.agent:main',
        ],
    },
)
