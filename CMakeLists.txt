cmake_minimum_required(VERSION 2.8.3)
project(configuration_node)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  message_generation
)

 add_message_files(
   FILES
   Param.msg
   NodeStatus.msg
   PackageStatus.msg
   NodesStatus.msg
   Scenario.msg
   NodeLaunchParam.msg
   NodeDetails.msg
)


 add_service_files(
   FILES
   GetSystemNodes.srv
   ActivateNodes.srv
   DeactivateNodes.srv
   GetDefaultScenario.srv
   GetParams.srv
   SetParam.srv
   CreateScenario.srv
   RestartSystem.srv
   SetDefaultScenario.srv
   RemoveScenarios.srv
   GetScenarios.srv
   GetNodesDetails.srv
   GenerateArduinoLibrary.srv
   StopRunningScenario.srv
   GetRobots.srv
   RunScenario.srv
   GetOOBENodes.srv
   SaveConfiguration.srv
   Shutdown.srv
   RestartOOBE.srv
   GetConfigurableNodes.srv
   RegisterNewNode.srv
   GetLaunchFilePath.srv
   FindLaunchFiles.srv
   FindRunningNodes.srv
   SetROSMasterURI.srv
   ExportScenarios.srv
   ImportScenarios.srv
   GetXMLData.srv
   UpdateXMLData.srv
   ReloadEuclidNode.srv
   StaticTransform.srv 	
   UpdateVersion.srv 
   EditScenario.srv
   DeleteNode.srv
   EditNode.srv
 )


# Install python scripts using distutils
catkin_python_setup()

 generate_messages(
   DEPENDENCIES
   std_msgs

 )

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
 CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS std_msgs rospy)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)



catkin_package(
    DEPENDS ${SYS_DEPS}
    CATKIN_DEPENDS ${CATKIN_DEPS}
)

catkin_install_python(PROGRAMS scripts/CsConfigModule.py scripts/CsConfigurationItem.py scripts/CsRegisterNewNode.py scripts/CsScenarioManager.py scripts/CsUtils.py 
                      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


