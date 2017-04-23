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
import subprocess
import os.path
import os
import glob

from xml.dom import minidom
import thread
import time
import cPickle
    
import rospkg 
import rosgraph 
import rosgraph.names 
from rosgraph.names import script_resolve_name 
   
import roslib.packages 
  
from roslaunch.core import setup_env, local_machine, RLException 
from roslaunch.config import load_config_default 
import roslaunch.xmlloader 


class CsRegisterNewNode:
    """CS main class, handles config.xml and system startup.
    
    This is the main cs class, it runs on startup,
    it handles scenarios/active scneario and all system nodes.
    """
    
    def __init__(self, configFilePath):
        """
        Constructor, takes a valid config.xml file
        """
        if not os.path.isfile(configFilePath):
            raise Exception(FileDoesntExist) 
        self._configurableNodes = []
        self._configFilePath = configFilePath

    def parseConfig(self):
        """ 
        config.xml parsing method for new nodes
    
        This method parses the config.xml and generates a suitable data structure
        from the xml file.
        Besides parsing the xml, it runs all system nodes.
        """
        tree = ET.parse(self._configFilePath)
        root = tree.getroot()  
        ###############################
        # parse nodes           #
        ###############################
        self._nodes = []
        self._activeNodes = []
        self._activeNodesPids = {}
        self._nodesData = {}
        self._runningScenario  = None
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
                print att.tag 
                print att.text

    def FindLaunchFiles(self, packageName):   
        """
        This method finds the launchfile in a package and show the list to the user
        """
        #Find the package folder and navigate to the launch folder of the package
        rospack = rospkg.RosPack()
        launchFileList = []
        try:
            packageLocation = rospack.get_path(packageName)
            launchFileLocation = packageLocation + '/launch'
            os.chdir(launchFileLocation)
            #get a list of all the launch files in the folder and display them to the user to choose
            for file in glob.glob("*.launch"):
                rospy.loginfo(file)
                launchFileList.append(file)
            return launchFileList
        except rospkg.ResourceNotFound:
            rospy.logerr("Package not found")
            return launchFileList

    def GetLaunchFilePath(self, packageName, launchFileName ):
        """
        This method returns the full path fo the launchfile
        """
        rospack = rospkg.RosPack()
        packageLocation = rospack.get_path(packageName)
        launchFileLocation = packageLocation + '/launch'
        #make the launchfilepath
        launchFilePath = launchFileLocation +'/'+launchFileName 
        return launchFilePath

    def ResolvedName(self, node):
        """
        Resolves the names of the argumetns
        """
        if node.name: 
            # $(anon id) passthrough 
            if node.name.startswith('$'): 
                return node.name 
            else: 
                return rosgraph.names.ns_join(node.namespace, node.name) 
        else: 
           return None 

    def GetNodeList(self, config): 
        """ 
        Returns the list of running nodes 
        """ 
        l = [ self.ResolvedName(node) for node in config.nodes] + [self.ResolvedName(test) for test in config.tests] 
        # filter out unnamed nodes 
        return [x for x in l if x] 
         

    def FindRunningNodes(self, launchFilePath):
        """
        This methods makes a list of the running nodes in a launchfile
        """
        runningNodeList = []
        launchFileList = [launchFilePath]
        try: 
             loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False) 
             config = load_config_default(launchFileList, None, loader=loader, verbose=False, assign_machines=False)
             node_list = self.GetNodeList(config) 

             for node in node_list:
                runningNodeList.append(node[1:])

             rospy.loginfo(runningNodeList)
             return runningNodeList 
        except RLException as e: 
             rospy.logerr(str(e))
             return -1
        
    def RegisterNewNode(self, nodeName, packageName, runningNodeList, launchFileName):
        """
        This method adds a new node to the web UI
        """
        #add the Nodename, packageName, runningNodeList, and Launchfile name to the config.xml
        try:
            tree = ET.parse(self._configFilePath)
            root = tree.getroot()
            
            nodes = root.findall('nodes')
            node = ET.SubElement(nodes[0], "node",{'name':nodeName})
            
            ET.SubElement(node, "package").text=packageName
            ET.SubElement(node, "launch_file").text=launchFileName 
            ET.SubElement(node, "configurationItem").text="some_file"
            runningNodes = ET.SubElement(node, "running_nodes")
            
            for rosNode in runningNodeList:
                ET.SubElement(runningNodes, "node").text=rosNode
            
            oobeNodes = root.findall('oobe_nodes')
            oobeNode = ET.SubElement(oobeNodes[0], "node").text=nodeName
            xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
            outputFile = self._configFilePath
            with open(outputFile,"w") as f:
                f.write(xmlstr)
            f.close()
            #self._nodes.append(nodeName)
            #self._nodesData[nodeName] = {}
            #   for att in node:
            #       if att.tag == 'running_nodes':
            #           nodesList = []
            #            for runningNode in att:
            #               nodesList.append(runningNode.text)
            #            self._nodesData[node.get('name')][att.tag] = nodesList
            #       else:
            #           self._nodesData[node.get('name')][att.tag] = att.text
            self.parseConfig()
            rospy.loginfo("New Node is registered successfully to the WebUI")
            return True
        except (len(runningNodeList) == 0):
            rospy.logerr("no running nodes selected")
            return False
        except IOError:
            rospy.logerr("Cannot open/write to the xml file")
            return False
    
def handle_findLaunchFiles(req):   
    return FindLaunchFilesResponse(manager.FindLaunchFiles(req.packageName))

def handle_getLaunchFilePath(req):
    return GetLaunchFilePathResponse(manager.GetLaunchFilePath(req.packageName, req.launchFileName))

def handle_findRunningNodes(req):
    return FindRunningNodesResponse(manager.FindRunningNodes(req.launchFilePath))
    
def handle_registerNewNode(req):
    return RegisterNewNodeResponse(manager.RegisterNewNode(req.nodeName, req.packageName, req.runningNodeList, req.launchFileName))
    
def startROSAPI():
    print "Activating nodes service"
    
    rospy.Service('/cs/find_launch_files',FindLaunchFiles,handle_findLaunchFiles)
    rospy.Service('/cs/get_launch_file_path',GetLaunchFilePath,handle_getLaunchFilePath)
    rospy.Service('/cs/find_running_nodes',FindRunningNodes,handle_findRunningNodes)
    rospy.Service('/cs/register_new_node',RegisterNewNode,handle_registerNewNode)
    
if __name__ == '__main__':
    try:
        rospy.init_node('CsNodeRegistrationNode')
        # configFilePath = rospy.get_param('configFilePath', "")
        manager = CsRegisterNewNode("/intel/euclid/config/config.xml");
        startROSAPI()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass