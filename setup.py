from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(
    name='configuration_node',
    version='1.0.0',
    scripts=['scripts/CsConfigModule.py', 'scripts/CsConfigurationItem.py', 'scripts/CsRegisterNewNode.py', 'scripts/CsScenarioManager.py', 'scripts/CsUtils.py'],
    packages=['configuration_node'],
    package_dir={'': '/intel/euclid/euclid_ws/src/system_nodes/'}
)


