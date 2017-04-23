#!/usr/bin/env python

##################################################################################
#Copyright (c) 2016, Intel Corporation
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this
#list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#this list of conditions and the following disclaimer in the documentation
#and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors
#may be used to endorse or promote products derived from this software without
#specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##################################################################################

import rospy
import xml.etree.ElementTree as ET
from configuration_node.srv import *
from configuration_node.msg import *
from dynamic_reconfigure.msg import Config
from dynamic_reconfigure.srv import Reconfigure
import subprocess
import os.path
import os
import sys

from xml.dom import minidom
import CsUtils
import thread
import time
import cPickle


TOPIC_PREFIX = "/cs"
NODE_PREFIX = "scenario_manager"
manager = ''
class CsScenarioManager:
    
    def __init__(self, configFilePath,scenarioName):
        """Constructor, takes a valid config.xml file
        
        """
        if not os.path.isfile(configFilePath):
            raise Exception(FileDoesntExist) 
            
        self._scenarioName = scenarioName
        self._configFilePath = configFilePath
        self._isRunning = False
        
        self._parseConfigFile()
        self._runScenario()
        
        thread.start_new_thread(self._scenarioMonitor, ())
        self._parameterListener()
     
    def _parseConfigFile(self):
        rospy.loginfo ('parsing config: ' + self._configFilePath)
        tree = ET.parse(self._configFilePath)
        root = tree.getroot()
        
        #######################
        self._nodes = []
        self._activeNodes = []
        self._activeNodesPids = {}
        self._nodesData = {}
        nodes = root.findall('nodes')
        for node in nodes[0]:
            self._nodes.append(node.get('name'))
            self._nodesData[node.get('name')] = {}
            for att in node:
                if att.tag == 'running_nodes':
                    nodesList = []
                    for runningNode in att:
                        nodesList.append(runningNode.text)
                    self._nodesData[node.get('name')][att.tag] = nodesList
                else:
                    self._nodesData[node.get('name')][att.tag] = att.text

        ###############################
        # parse scenarios             #
        ###############################
        scenarios = root.findall('scenarios')
        self._scenarios = {}
        for scenario in scenarios[0]:
            self._scenarios[scenario.get('name')] = {}
            self._scenarios[scenario.get('name')]['launch_parameters'] = {}
            self._scenarios[scenario.get('name')]['name'] = scenario.get('name')
            nodeList = []
            self._scenarios[scenario.get('name')]['nodes'] = []
            for node in scenario.findall('node'):
                nodeList.append(node.get('name'))
                params = node.findall('launch_parameters')
                self._scenarios[scenario.get('name')]['launch_parameters'][node.get('name')] = params[0].text
            self._scenarios[scenario.get('name')]['nodes'] = nodeList
        
        
        
    def _runScenario(self):
        print 'running scenario: ' + self._scenarioName

        if self._isRunning:
            rospy.logerr('Scenario is already running')
            return
        
        if self._scenarioName  in self._scenarios:
            for node in self._scenarios[self._scenarioName]['nodes']:
                
                print "Running node: " + node
                
                self._runNode(node,self._scenarios[self._scenarioName]['launch_parameters'][node])		
                time.sleep(1)
            self._loadScenarioConfiguration()
        else:
          print "Scenario not found"
          sys.exit(0)
        
        
    def _runNode(self, nodeName,params):
        """

        The method gets a nodeName and runs it
        """
        if nodeName not in self._nodesData:
            return
        print "running node: " + nodeName
        
        #string = '/intel/euclid/oobe-utils/launchScript.sh roslaunch ' + self._nodesData[nodeName]['package'] + ' ' + self._nodesData[nodeName]['launch_file']
        if params == None:
            params = self._nodesData[nodeName]['launch_parameters']
        
        

        print "Running params: " + str(params)
        self._activeNodes.append(nodeName)
        #print "Command: " + "roslaunch " + self._nodesData[nodeName]['package'] + " " + self._nodesData[nodeName]['launch_file'] + " " + params
        if params == None:
            proc = subprocess.Popen(['roslaunch', self._nodesData[nodeName]['package'],self._nodesData[nodeName]['launch_file']] )
        else:
            proc = subprocess.Popen(['roslaunch', self._nodesData[nodeName]['package'],self._nodesData[nodeName]['launch_file']] + params.split(" "))
        self._activeNodesPids[nodeName] = proc.pid
        return True
        
    def _stopNode(self, nodeName):
        if nodeName in self._activeNodes:
            print "Killing pid: " + str(self._activeNodesPids[nodeName])
            subprocess.Popen(['kill', str(self._activeNodesPids[nodeName])])
            self._activeNodes.remove(nodeName)
            if nodeName in self._activeNodesPids: del self._activeNodesPids[nodeName]

            return True
            
        return False
    
    
    def _scenarioMonitor(self):  
        """ Monitors running nodes
        
        This method checks every running node whether it is running or not
        then it publishes a ROS topic with every node status.
        """
        nodesPublisher = rospy.Publisher('/euclid/scenario_status',NodesStatus,queue_size=1)
        #TODO: constants should be defined above.
        while True:
            status = []
            for node in self._activeNodes:
                packageStatus = []
                for runningNode in self._nodesData[node]['running_nodes']:
                    isRunning = CsUtils.IsNodeRunning(runningNode)
                    packageStatus.append(NodeStatus(runningNode,isRunning))
                
                status.append(PackageStatus(node,packageStatus))
            nodesPublisher.publish(NodesStatus(status))
            time.sleep(0.1) 
            
    
    def _parameterListener(self):
        print "=============================================================="
        print "Parameter listener!"
        self._scenarioConfigurableNodes = {}
        self._scenarioSavedNodesData = {}
        self._loadedNodesData = {}
        directory = "/intel/euclid/config/scenarios/" + self._scenarioName 
        for node in self._activeNodes:
            for runningNode in self._nodesData[node]['running_nodes']:    
                self._scenarioConfigurableNodes[runningNode] = ''
                self._scenarioSavedNodesData[runningNode] = ''
                self._loadedNodesData[runningNode] = False
                objFile = directory + '/' + runningNode.replace("/","_") + ".p"
                rospy.loginfo('Checking file: ' + objFile)
                if os.path.exists(objFile):
                    rospy.loginfo('Reading scenario data: ' + objFile)
                    self._scenarioSavedNodesData[runningNode] = cPickle.load(open(objFile,'rb'))
                    
                rospy.Subscriber(runningNode + "/parameter_updates",Config,self._nodeParameterCallback,runningNode)
                
    
    
    def _nodeParameterCallback(self,data,node):
        print "Got Parameter: " + node
        self._scenarioConfigurableNodes[node] = data
        if not self._loadedNodesData[node] and self._scenarioSavedNodesData[node] != '':
            self._loadedNodesData[node] = True
            print "=============================================================="
            print "Got Parameter"
            serviceCall = rospy.ServiceProxy('/'+node + '/set_parameters',Reconfigure)
            serviceCall(self._scenarioSavedNodesData[node])
        
   
    def _loadScenarioConfiguration(self):
        print "todo"
    
   
    def SaveScenario(self,req):
        directory = "/intel/euclid/config/scenarios/" + self._scenarioName 
        if not os.path.exists(directory):
            os.makedirs(directory)
        for node in self._scenarioConfigurableNodes:
            print "Saving: " + node  + ": " + "/home/ros/" + node + ".p"
            cPickle.dump(self._scenarioConfigurableNodes[node], open(directory +  '/' + node.replace("/","_") + ".p","wb"))    
        return True    
            
            
    def KillScenario(self):
        rospy.loginfo('Shutting down scenario: ' + self._scenarioName)
        for node in self._scenarios[self._scenarioName]['nodes']:
            self._stopNode(node)
        if '_transform' in self._activeNodesPids:
            self._stopNode('_transform')
      
    def ReloadNode(self,req):
        if req.nodeName in self._activeNodes:
            self._stopNode(node)
            self._runNode(node,self._scenarios[self._scenarioName]['launch_parameters'][node])		
            return True
        return False
    def StaticTransform(self,req):
        proc = subprocess.Popen(['rosrun','tf','static_transform_publisher',req.x,req.y,req.z,req.yaw,req.pitch,req.roll,req.frame_id,req.child_frame_id])
        self._activeNodesPids['_transform'] = proc.pid
        
def onShutdown():
    rospy.loginfo('Shutdown scenario')
    manager.KillScenario()

       
def startROSAPI():

   
    rospy.Service('/euclid/save_configuration',SaveConfiguration,manager.SaveScenario)
    rospy.Service('/euclid/reload_euclid_node',ReloadEuclidNode,manager.ReloadNode)
    rospy.Service('/euclid/publish_static_transform',StaticTransform,manager.StaticTransform)
    
if __name__ == '__main__':
    try:
        
        rospy.init_node('CsScenarioManager')
        scenarioName =  str(sys.argv[1])
        if scenarioName == 'NONE':
            rospy.logerr( "Error - Scenario manager ran without a scenario name")
            sys.exit(0)
       
        
        manager = CsScenarioManager("/intel/euclid/config/config.xml",scenarioName);
        
        rospy.on_shutdown(onShutdown)
        startROSAPI()
        rospy.spin()
        
        
    except rospy.ROSInterruptException:
        print "Interrupt!!!"
        pass
   
