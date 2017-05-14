# Intel&reg; Euclid&trade; Configuration node.

This system node is reponsible for handling Nodes, Scenarios and loading of their configuration on the device. 

http://www.intel.com/Euclid_XXX

http://wiki.ros.org/EuclidWiki_XXX

## Subscribed Topics

    None

## Published Topics

    nodes_status 
        publishes a ROS topic with status of every running node.
		
## Parameters
    None
		
## Services
    /euclid/get_system_nodes (configuration_node/GetSystemNodes)
        Gives a list of running system nodes
    /euclid/activate_nodes (configuration_node/ActivateNodes)
        Gets a node name and runs it
    /euclid/deactivate_nodes (configuration_node/DeactivateNodes)
        Stops a running scenario
    /euclid/stop_running_scenario (configuration_node/StopRunningScenario)
        Stops the Running Scenario from Euclid
    /euclid/get_default_scenario (configuration_node/GetDefaultScenario)
        Get the Default running scenario for Euclid
    /euclid/get_params (configuration_node/GetParams)
        Gets the parameters for the nodes
    /euclid/get_configurable_nodes (configuration_node/GetConfigurableNodes)
        Returns a list of configurable Euclid Nodes
    /euclid/set_param (configuration_node/SetParam)
        Set Parameters for a scenario
    /euclid/create_scenario (configuration_node/CreateScenario)
        Creates a new scenario
    /euclid/restart_system (configuration_node/RestartSystem)
        Reboots Euclid
    /euclid/shutdown_system (configuration_node/ShutdownSystem)
        Shuts down Euclid
    /euclid/restart_oobe (configuration_node/RestartOOBE)
        Restarts the Web UI and the background processes runing on Euclid
    /euclid/set_default_scenario (configuration_node/SetDefaultScenario)
        Sets the specified scenario to be default
    /euclid/remove_scenarios (configuration_node/RemoveScenarios)
        Deletes the specified Scenario from the Web UI
    /euclid/get_scenarios (configuration_node/GetScenarios)
        Returns a list of Scenarios from the existing on the Web UI
    /euclid/generate_arduino_library (configuration_node/GenerateArduinoLibrary)
        Generates Arduino Library to be downloaded on the host device where the web interface is accessed
    /euclid/get_oobe_nodes (configuration_node/GetOOBENodes)
        Returns a list of Euclid Automation Node
    /euclid/get_robots (configuration_node/GetRobots)
        Returns a list of robots
    /euclid/run_scenario (configuration_node/RunScenario)
        Runs a specific scenario as specified by the user
    /euclid/set_ros_master_uri (configuration_node/SetROSMasterURI)
        Set the ROS_MASTER_URI of the device
    /euclid/export_scenario (configuration_node/ExportScenario)
        Exports the Scenario data from the Web UI
    /euclid/import_scenario (configuration_node/ImportScenario)
        Imports a Scenario to the Web UI
    /euclid/get_xml_data (configuration_node/GetXMLData)
        returns the xml data of a scenario or a euclid node
    /euclid/update_xml_data (configuration_node/UpdateXMLData)
        Update Scenario data or the Euclid node data on the Web UI
    /euclid/find_launch_files (configuration_node/FindLaunchFiles)
        Finds the list of launch files in a package
    /euclid/get_launch_file_path (configuration_node/GetLaunchFilePath)
        Returns a path for a launchfile in a specified package
    /euclid/find_running_nodes (configuration_node/FindRunningNodes)
        Returns a list of Running Nodes in a Launch File runs on execution
    /euclid/register_new_node (configuration_node/RegisterNewNode)
        Registers a new Node to the Web UI
    /euclid/update_version (configuration_node/UpdateVersion)
        Updates the version of the Euclid Automation Layer
    /euclid/edit_scenario (configuration_node/EditScenario)
        Edits an exiting scenario in the Web UI
    /euclid/delete_node (configuration_node/DeleteNode)
        Deletes a node from the Web UI
    /euclid/edit_node (configuration_node/EditNode)
        Edits an existing node in the Web UI
    /euclid/get_nodes_details (configuration_node/GetNodesDetails)
    
## Contributing to the Project

The Intel&reg; Euclid&trade; Configuration node is developed and distributed under
a BSD-3 license as noted in [licenses/License.txt](licenses/License.txt).

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
have the right to submit it under the open source license
indicated in the file; or

(b) The contribution is based upon previous work that, to the best
of my knowledge, is covered under an appropriate open source
license and I have the right under that license to submit that
work with modifications, whether created in whole or in part
by me, under the same open source license (unless I am
permitted to submit under a different license), as indicated
in the file; or

(c) The contribution was provided directly to me by some other
person who certified (a), (b) or (c) and I have not modified
it.

(d) I understand and agree that this project and the contribution
are public and that a record of the contribution (including all
personal information I submit with it, including my sign-off) is
maintained indefinitely and may be redistributed consistent with
this project or the open source license(s) involved.

## Configuration:

| Version        | Best Known           |
|:-------------- |:---------------------|
| OS             | Ubuntu 16.04 LTS     |
| ROS            | Kinetic              |
