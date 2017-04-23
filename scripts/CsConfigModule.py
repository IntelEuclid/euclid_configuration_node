#!/usr/bin/env python

##########################################################################
# Copyright (c) 2016, Intel Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##########################################################################

import rospy
import xml.etree.ElementTree as ET
from configuration_node.srv import *
from configuration_node.msg import *
from dynamic_reconfigure.msg import Config
import subprocess
import os.path
import os

from xml.dom import minidom
import CsUtils
import thread
import time
import cPickle
import glob
import re

import rospkg
import rosgraph
import rosgraph.names
from rosgraph.names import script_resolve_name

import roslib.packages

from roslaunch.core import setup_env, local_machine, RLException
from roslaunch.config import load_config_default
import roslaunch.xmlloader
import CsOobeHelper


class CsConfigModule:
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
        self.parseConfig()
        self.RunDefaultScenario()
        self._runNodesMonitoringThread()
        subprocess.Popen(
            ['python /intel/euclid/oobe-services/startup_services/S6FeedbackServer/soundClient.py P\$"/usr/share/sounds/ubuntu/ringtones/Supreme.ogg"'], shell=True)

    def _runNodesMonitoringThread(self):
        """ runs a thread that monitors all running nodes
        """
        thread.start_new_thread(self.nodesMonitor, ())
        thread.start_new_thread(self.configurableNodesMonitor, ())

    # TODO check if 5 sec is enough/necessary?
    def configurableNodesMonitor(self):
        self._configurableNodes = []
        while True:
            monitor = subprocess.Popen(
                ["rosrun dynamic_reconfigure dynparam list"], stdout=subprocess.PIPE, shell=True)
            (data, errors) = monitor.communicate()
            nodesList = data.split("\n")
            self._configurableNodes = nodesList[0:len(nodesList) - 1]
            time.sleep(5)

    def nodesMonitor(self):
        """ Monitors running nodes

        This method checks every running node whether it is running or not
        then it publishes a ROS topic with every node status.
        """
        nodesPublisher = rospy.Publisher(
            'nodes_status', NodesStatus, queue_size=1)
        # TODO: constants should be defined above.
        while True:
            if (nodesPublisher.get_num_connections() > 0):
                status = []
                for node in self._activeNodes:
                    packageStatus = []
                    for runningNode in self._nodesData[node]['running_nodes']:
                        isRunning = CsUtils.IsNodeRunning(runningNode)
                        packageStatus.append(
                            NodeStatus(runningNode, isRunning))

                    status.append(PackageStatus(node, packageStatus))
                nodesPublisher.publish(NodesStatus(status))
            time.sleep(1)

    def parseConfig(self):
        """ config.xml parsing method
            ws
        This method parses the config.xml and generates a suitable data structure
        from the xml file.
        Besides parsing the xml, it runs all system nodes.
        """
        try:
            tree = ET.parse(self._configFilePath)
        except Exception:
            self._configFilePath = '/intel/euclid/config/default_config.xml'
            print "Bad xml file, falling back to default"
            tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        ###############################
        # parse nodes           #
        ###############################
        self._nodes = []
        self._activeNodes = []
        self._activeNodesPids = {}
        self._nodesData = {}
        self._runningScenario = None
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
                # print att.tag
                # print att.text

        ###############################
        # parse scenarios             #
        ###############################
        scenarios = root.findall('scenarios')
        self._scenarios = {}
        for scenario in scenarios[0]:
            self._scenarios[scenario.get('name')] = {}
            self._scenarios[scenario.get('name')][
                'name'] = scenario.get('name')
            nodeList = []
            self._scenarios[scenario.get('name')]['nodes'] = []
            self._scenarios[scenario.get('name')]['launch_parameters'] = {}
            for node in scenario.findall('node'):
                nodeList.append(node.get('name'))
                params = node.findall('launch_parameters')
                self._scenarios[scenario.get('name')]['launch_parameters'][
                    node.get('name')] = params[0].text

            self._scenarios[scenario.get('name')]['nodes'] = nodeList

        ###############################
        # parse default scenario       #
        ###############################
        defaultScenario = root.findall('default_scenario')
        if defaultScenario == '' or defaultScenario == []:
            self._defaultScenario = None
        else:
            self._defaultScenario = defaultScenario[0].find('name').text

        ###############################
        # parse oobe-nodes            #
        ###############################
        oobe_nodes = root.findall('oobe_nodes')
        self._oobe_nodes = []
        for oobe_node in oobe_nodes[0]:
            self._oobe_nodes.append(oobe_node.get('name'))

        ###############################
        # parse robots                #
        ###############################
        robots = root.findall('robots')
        self._robots = []
        for robot in robots[0]:
            self._robots.append(robot.text)

        ###############################
        # parse and run system nodes  #
        ###############################
        systemNodes = root.findall('system_nodes')
        self._system_nodes = {}
        nodeList = []
        for node in systemNodes[0].findall('node'):
            nodeList.append(node.text)
            self.ActivateNode(node.text)
        self._system_nodes = nodeList

    def GetRobots(self):
        return self._robots

    def GetOOBENodes(self):
        return self._oobe_nodes + self._robots

    def GetRunningScenario(self):
        return self._runningScenario

    def ActivateNode(self, nodeName):
        """

        The method gets a nodeName and runs it
        """
        string = 'roslaunch ' + \
            self._nodesData[nodeName]['package'] + ' ' + \
            self._nodesData[nodeName]['launch_file']
        self._activeNodes.append(nodeName)
        proc = subprocess.Popen(
            ['/intel/euclid/oobe-utils/launchScript.sh', string])
        self._activeNodesPids[nodeName] = proc.pid
        return True

    def GetActiveNodes(self):
        """ Returns active (running) nodes
        """
        return self._activeNodes

    def GetAllNodes(self):
        """ Returns all system nodes
        """
        return self._nodes

    def validNodeName(self, nodeName):
        if nodeName == "":
            return False
        search = re.compile(r'[^a-z0-9A-Z_\-]').search
        if bool(search(nodeName)):
            return False
        return True

    def RegisterNewNode(self, nodeName, packageName, runningNodeList, launchFileName, launchParams):
        """
        This method adds a new node to the web UI
        """
        #if not self.validNodeName(nodeName):
            #return False

        if nodeName in self._nodes:
            return False

        self._nodes.append(nodeName)
        self._nodesData[nodeName] = {}
        self._nodesData[nodeName]['package'] = packageName
        self._nodesData[nodeName]['running_nodes'] = runningNodeList
        self._nodesData[nodeName]['launch_file'] = launchFileName
        self._nodesData[nodeName]['configurationItem'] = 'some_file'
        self._nodesData[nodeName]['launch_parameters'] = launchParams
        self._oobe_nodes.append(nodeName)

        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        nodes = root.findall('nodes')
        node = ET.SubElement(nodes[0], "node", {'name': nodeName})

        ET.SubElement(node, "package").text = packageName
        ET.SubElement(node, "launch_file").text = launchFileName
        ET.SubElement(node, "configurationItem").text = "some_file"
        ET.SubElement(node, "launch_parameters").text = launchParams
        runningNodes = ET.SubElement(node, "running_nodes")

        for rosNode in runningNodeList:
            ET.SubElement(runningNodes, "node").text = rosNode

        oobeNodes = root.findall('oobe_nodes')
        oobeNode = ET.SubElement(oobeNodes[0], "node", {'name': nodeName})
        xmlstr = minidom.parseString(
            ET.tostring(root)).toprettyxml(indent="  ")
        outputFile = self._configFilePath
        with open(outputFile, "w") as f:
            f.write(xmlstr)
        f.close()
        return True

    def EditNode(self, nodeName, packageName, runningNodeList, launchFileName, launchParams):
        """
        This method will edit the specified node
        """
        # Delete the old Node and create the new one with new parameters

        try:
            #if self.validNodeName(nodeName) == False:
                #return False
            if self.DeleteNode(nodeName, True) == 0:
                print("deleted Node");
                return self.RegisterNewNode(nodeName, packageName, runningNodeList, launchFileName, launchParams)
            return False
        except Exception as e:
            rospy.logerr("Unable to Edit Node")
            print(e)
            return False

    def DeleteNode(self, nodeName, force=False):
        """
        This method will delete the specified node
        """
        # add the Nodename, packageName, runningNodeList, and Launchfile name
        # to the config.xml 
        try:
            tree = ET.parse(self._configFilePath)
            root = tree.getroot()
            nodes = root.findall('nodes')
            scenarios = root.findall('scenarios')
            result = -1
            # Check if node is used in another scenario if so, the return error
            if not force:
                for scenario in scenarios[0].findall('scenario'):
                    for node in scenario.findall('node'):
                        if node.get('name') == nodeName:
                            rospy.logerr(
                                "Node is used in Scenario, please delete the scenario first")
                            return -2
            # for all node element if node Name == nodeName, Delete that node.
            for node in nodes[0].findall('node'):
                if node.get('name') == nodeName:
                    nodes[0].remove(node)
                    result = 0
                    break
            oobeNodes = root.findall('oobe_nodes')
            for node in oobeNodes[0].findall('node'):
                if node.get('name') == nodeName:
                    oobeNodes[0].remove(node)
                    result = 0
                    break
            if result == 0:
                xmlstr = minidom.parseString(
                    ET.tostring(root)).toprettyxml(indent="  ")
                outputFile = self._configFilePath
                with open(outputFile, "w") as f:
                    f.write(xmlstr)
                f.close()
                self.parseConfig()
                rospy.loginfo("Node deleted successfully from the WebUI")
            if result == -1:
                rospy.logerr("Node not found")
            return result
        except IOError:
            rospy.logerr("Cannot open/write to the xml file")
            return -3

    def GetDefaultScenario(self):
        """Returns active scenario
        """
        return self._defaultScenario

    def CreateScenario(self, scenarioName, nodesList, nodesLaunchParams):
        """ Creates a new scenario in config.xml
        """

        if scenarioName in self._scenarios:
            print "Scenario already exists"
            return False

        if len(nodesList) == 0:
            print "Nodelist is empty"
            return False

        for node in nodesList:
            if node not in self._oobe_nodes and node not in self._robots:
                print "Node: " + node + " not found"
                return False

        search = re.compile(r'[^a-z0-9A-Z_\-]').search
        if bool(search(scenarioName)):
            return False

        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        scenarios = root.findall('scenarios')
        scenario = ET.SubElement(scenarios[0], "scenario", {
                                 'name': scenarioName})

        for node in nodesList:
            nodeElem = ET.SubElement(scenario, "node", {'name': node})
            if node in nodesLaunchParams:
                ET.SubElement(
                    nodeElem, "launch_parameters").text = nodesLaunchParams[node]
            else:
                ET.SubElement(nodeElem, "launch_parameters")

        #xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
        xmlstr = ET.tostring(root)
        outputFile = self._configFilePath
        with open(outputFile, "w") as f:
            f.write(xmlstr)
        f.close()
        self._scenarios[scenarioName] = {}
        self._scenarios[scenarioName]['name'] = scenarioName
        self._scenarios[scenarioName]['nodes'] = nodesList
        self._scenarios[scenarioName]['launch_parameters'] = {}
        for node in nodesList:
            if node in nodesLaunchParams:
                self._scenarios[scenarioName]['launch_parameters'][
                    node] = nodesLaunchParams[node]
            else:
                self._scenarios[scenarioName]['launch_parameters'][node] = None

        return True

    def EditScenario(self, scenarioName, newScenarioName, nodesList, nodesLaunchParams):
        print "editing scenario"
        if scenarioName in self._scenarios:

            # rename underlying configuration
            oldDirectory = "/intel/euclid/config/scenarios/" + scenarioName
            newDirectory = "/intel/euclid/config/scenarios/" + newScenarioName
            if os.path.exists(oldDirectory):
                os.rename(oldDirectory, newDirectory)

            self.RemoveScenario(scenarioName)
            self.CreateScenario(newScenarioName, nodesList, nodesLaunchParams)
            return True
        return False

    def RunScenario(self, scenarioName):
        if self._runningScenario is not None:
            self.StopRunningScenario()
        if scenarioName in self._scenarios:
            self._runningScenario = scenarioName
            string = '/intel/euclid/oobe-utils/launchScript.sh roslaunch configuration_node scenario_manager.launch scenario:=' + scenarioName

            subprocess.Popen(
                ['python /intel/euclid/oobe-services/startup_services/S6FeedbackServer/client.py R0\$1110000'], shell=True)
            subprocess.Popen(
                ['python /intel/euclid/oobe-services/startup_services/S6FeedbackServer/soundClient.py P\$"/usr/share/sounds/ubuntu/ringtones/Bliss.ogg"'], shell=True)

            proc = subprocess.Popen(['su', 'euclid', '-c', string])
            self._scenarioPid = proc.pid
            return True

        return False

    def RunDefaultScenario(self):
        """ Runs active scenario
        """
        if self._defaultScenario is not None:
            self.RunScenario(self._defaultScenario)

    def StopRunningScenario(self):
        if self._runningScenario is not None:

            subprocess.Popen(
                ['python /intel/euclid/oobe-services/startup_services/S6FeedbackServer/client.py R5\$101010'], shell=True)
            subprocess.Popen(
                ['python /intel/euclid/oobe-services/startup_services/S6FeedbackServer/soundClient.py P\$"/usr/share/sounds/ubuntu/notifications/Xylo.ogg"'], shell=True)

            self._runningScenario = None
            subprocess.Popen(['kill', str(self._scenarioPid)])

    def RemoveDefaultScenario(self):
        """ After calling this method, there will be no active scenario
        """
        self.SetDefaultScenario("")
        return True

    def SetDefaultScenario(self, scenarioName):
        """ Sets a scenario as active (on startup)
        """
        if scenarioName != "" and scenarioName not in self._scenarios:
            return False

        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        default_scenario = root.findall('default_scenario')
        activeScenarioName = default_scenario[0].findall('name')

        activeScenarioName[0].text = scenarioName

        xmlstr = ET.tostring(root)
        outputFile = self._configFilePath
        with open(outputFile, "w") as f:
            f.write(xmlstr)
        f.close()

        self._defaultScenario = scenarioName

        return True

    def RemoveScenario(self, scenarioName):
        """ Removes a scenario from the config.xml
        """

        # WORKAROUND: scenario name could start with space when numeric.
        scenarioName = scenarioName.strip()
        if scenarioName not in self._scenarios:
            print "Scenario doesn't exist"
            return False

        if self._runningScenario == scenarioName:
            self.StopRunningScenario()
        if self._defaultScenario == scenarioName:
            self.RemoveDefaultScenario()

        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        scenarios = root.findall('scenarios')
        for scenario in scenarios[0]:
            if scenario.get('name') == scenarioName:
                scenarios[0].remove(scenario)
                break

        del self._scenarios[scenarioName]
        xmlstr = ET.tostring(root)
        outputFile = self._configFilePath
        with open(outputFile, "w") as f:
            f.write(xmlstr)
        f.close()
        print "Removing scenario data"
        subprocess.call(
            ['rm', '-rf', '/intel/euclid/config/scenarios/' + scenarioName])
        print "Done removing scenario"
        return True

    def GetScenarios(self):
        # print "I was called"
        names = []
        for scenario in self._scenarios:
            # print "Getting scenario: " + scenario
            names.append(self._scenarios[scenario]['name'])
	
        return names

    def GetScenarioNodes(self, scenarioName):
        if scenarioName in self._scenarios:
            return self._scenarios[scenarioName]['nodes']
        else:
            return []

    def getScenarioData(self, scenarioName):
        for scenario in self._scenarios:
            if self._scenarios[scenario]['name'] == scenarioName:
                return self._scenarios[scenario]
                break

        return ''

    def getScenarioDataString(self, scenarioName):
        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        scenarios = root.findall('scenarios')
        for scenario in scenarios[0]:

            if scenario.get('name') == scenarioName:
                return ET.tostring(scenario)
                break

        return ''

    def getEuclidNodeData(self, nodeName):
        tree = ET.parse(self._configFilePath)
        root = tree.getroot()
        nodes = root.findall('nodes')
        for node in nodes[0]:
            if node.get('name') == nodeName:
                return ET.tostring(node)
                break

        return ''

    def updateScenario(self, scenarioName, data):
        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        scenarios = root.findall('scenarios')
        for scenario in scenarios[0]:

            if scenario.get('name') == scenarioName:
                scenarios[0].remove(scenario)
                scenarios[0].append(ET.fromstring(data))
        xmlstr = minidom.parseString(
            ET.tostring(root)).toprettyxml(indent="  ")
        outputFile = self._configFilePath
        with open(outputFile, "w") as f:
            f.write(xmlstr)
        f.close()

    def updateEuclidNode(self, nodeName, data):
        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        nodes = root.findall('nodes')
        for node in nodes[0]:

            if node.get('name') == nodeName:
                nodes[0].remove(node)
                nodes[0].append(ET.fromstring(data))
        xmlstr = minidom.parseString(
            ET.tostring(root)).toprettyxml(indent="  ")
        outputFile = self._configFilePath

        with open(outputFile, "w") as f:
            f.write(xmlstr)
        f.close()

    def exportScenarios(self, scenariosList):
        root = ET.Element('scenarios')
        for scenario in scenariosList:
            data = self.getScenarioDataString(scenario)
            if data != '':

                root.append(ET.fromstring(data))
        return ET.tostring(root)

    def importScenarios(self, scenariosData):

        scenariosAdded = []
        newroot = ET.fromstring(scenariosData)

        newScenarios = newroot.findall('scenario')
        tree = ET.parse(self._configFilePath)
        root = tree.getroot()

        scenarios = root.findall('scenarios')
        for newScenario in newScenarios:
            if newScenario.get('name') not in self._scenarios:
                print "Adding: " + str(newScenario.get('name'))
                scenarios[0].append(ET.fromstring(ET.tostring(newScenario)))
                scenariosAdded.append(newScenario.get('name'))

        #xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
        outputFile = self._configFilePath

        with open(outputFile, "w") as f:
            f.write(ET.tostring(root))
        f.close()
        return scenariosAdded

    def GetConfigurableNodes(self):
        return self._configurableNodes

    def UpdateVersion(self):
        string = '/intel/euclid_update/sw_update.bash'
        try:
            proc = subprocess.check_call(
                ['bash', string], stderr=subprocess.PIPE)
        except subprocess.CalledProcessError:
            print 'Sw update ended with Error'
            return False
        print 'Sw update ended with Success'
        return True

    def FindLaunchFiles(self, packageName):
        """
        This method finds the launchfile in a package and show the list to the user
        """
        # Find the package folder and navigate to the launch folder of the
        # package
        rospack = rospkg.RosPack()
        launchFileList = []
        try:
            packageLocation = rospack.get_path(packageName)
            launchFileLocation = packageLocation + '/launch'
            os.chdir(launchFileLocation)
            # get a list of all the launch files in the folder and display them
            # to the user to choose
            for file in glob.glob("*.launch"):
                rospy.loginfo(file)
                launchFileList.append(file)
            return launchFileList
        except rospkg.ResourceNotFound:
            rospy.logerr("Package not found")
            return launchFileList

    def GetLaunchFilePath(self, packageName, launchFileName):
        """
        This method returns the full path fo the launchfile
        """
        rospack = rospkg.RosPack()
        packageLocation = rospack.get_path(packageName)
        launchFileLocation = packageLocation + '/launch'
        # make the launchfilepath
        launchFilePath = launchFileLocation + '/' + launchFileName
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
        l = [self.ResolvedName(node) for node in config.nodes] + \
            [self.ResolvedName(test) for test in config.tests]
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
            config = load_config_default(
                launchFileList, None, loader=loader, verbose=False, assign_machines=False)
            node_list = self.GetNodeList(config)

            for node in node_list:
                runningNodeList.append(node[1:])

            rospy.loginfo(runningNodeList)
            return runningNodeList
        except RLException as e:
            rospy.logerr(str(e))
            return -1

    def GetNodesDetails(self):
        """
        This method returns a detailed list of all the nodes
        """
        nodes = []
        for nodeName in self._oobe_nodes:
            launchParams = self._nodesData[nodeName]['launch_parameters']
            packageName = self._nodesData[nodeName]['package']
            runningNodeList = self._nodesData[nodeName]['running_nodes']
            launchFileName = self._nodesData[nodeName]['launch_file']
            nodes.append(NodeDetails(nodeName, packageName,
                                     runningNodeList, launchFileName, launchParams))

        for nodeName in self._robots:
            launchParams = self._nodesData[nodeName]['launch_parameters']
            packageName = self._nodesData[nodeName]['package']
            runningNodeList = self._nodesData[nodeName]['running_nodes']
            launchFileName = self._nodesData[nodeName]['launch_file']
            nodes.append(NodeDetails(nodeName, packageName,
                                     runningNodeList, launchFileName, launchParams))

        return nodes

# TODO: move to another file


def handle_getSystemNodes(req):
    if req.type == "active":
        nodes = manager.GetActiveNodes()
    elif req.type == "all":
        nodes = manager.GetAllNodes()
    else:
        nodes = []

    return GetSystemNodesResponse(nodes)


def handle_activateNode(req):
    for node in req.nodes:
        manager.ActivateNode(node)
    return True


def handle_deactiveNode(req):
    for node in req.nodes:
        manager.DeactivateNode(node)
    return True


def handle_getDefaultScenario(req):
    print manager.GetDefaultScenario()
    # if manager.GetDefaultScenario() == '':
    # print "empty.."

    # return ('None',['koko'])
    # if manager.GetScenarioNodes(manager.GetDefaultScenario()) == []:
    # print "empty list"
    return (manager.GetDefaultScenario(), manager.GetScenarioNodes(manager.GetDefaultScenario()))


def handle_getParams(req):
    dic = CsUtils.GetParams(req.nodeName)
    params = []
    for key in dic:
        params.append(Param(key, str(dic[key])))
    return GetParamsResponse(params)


def handle_exportScenarios(req):

    xmlstr = manager.exportScenarios(req.scenariosList)
    print "Writing data: " + str(xmlstr)
    dir = "/intel/euclid/public_html/files/save_data/" + \
        time.strftime("%d-%m-%y-%I-%H-%M-%W")
    os.makedirs(dir)
    os.makedirs(dir + "/config")
    with open(dir + "/scenarios.xml", "w") as f:
        f.write(xmlstr)
    f.close()
    for scenario in req.scenariosList:
        subprocess.call(
            ['cp', '-rf', '/intel/euclid/config/scenarios/' + scenario, dir + "/config/"])

    subprocess.call(['zip', '-r', 'saved.zip', '.'], cwd=dir)
    subprocess.call(['pwd'], cwd=dir)
    return dir + "/saved.zip"


def handle_importScenarios(req):
    if ".zip" in req.file:
        dir = "/tmp/" + time.strftime("%d-%m-%y-%I-%H-%M-%W")
        os.makedirs(dir)
        subprocess.call(['unzip', req.file, '-d', dir])
        with open(dir + '/scenarios.xml') as f:
            xmlstr = f.read()
        for config in glob.glob(dir + "/config/*"):
            # print "Copying: " + config
            subprocess.call(
                ['cp', '-rf', config, '/intel/euclid/config/scenarios/'])
    else:
        with open(req.file) as f:
            xmlstr = f.read()

    return manager.importScenarios(xmlstr)


def handle_getXMLData(req):
    if req.type == "scenario":
        return manager.getScenarioDataString(req.name)
    if req.type == "euclidNode":
        return manager.getEuclidNodeData(req.name)


def handle_updateXMLData(req):
    if req.type == "scenario":
        manager.updateScenario(req.data)
    if req.type == "euclidNode":
        manager.updateEuclidNode(req.data)
    return True


def handle_setParam(req):
    return SetParamResponse(CsUtils.SetParam(req.nodeName, req.param, req.value))


def handle_createScenario(req):
    params = {}
    for i in xrange(0, len(req.launchparams)):
        params[req.launchparams[i].node] = req.launchparams[i].param
    return manager.CreateScenario(req.scenarioName, req.nodes, params)


def handle_editScenario(req):
    params = {}
    for i in xrange(0, len(req.launchparams)):
        params[req.launchparams[i].node] = req.launchparams[i].param
    return manager.EditScenario(req.scenarioName, req.newScenarioName, req.nodes, params)

# TODO: password, hashed (md5?)


def handle_restartSystem(req):
    CsOobeHelper.Reboot()
    return True


def handle_shutdownSystem(req):
    CsOobeHelper.Shutdown()
    return True


def handle_setDefaultScenario(req):
    print "Got: " + req.scenarioName
    if req.scenarioName == "#NONE#":
        manager.RemoveDefaultScenario()
        return True
    else:
        return manager.SetDefaultScenario(req.scenarioName)


def handle_removeScenarios(req):
    res = True
    for scenarioName in req.scenarioName:
        b = manager.RemoveScenario(scenarioName)
        res = (res and b)
    return res

def scenarioCmp(x,y):
    #if x.isRunning:
	#return -1
    #if y.isRunning:
    #    return 1
    #if x.isActive:
	#return -1
    #if y.isActive:
    #    return 1
    return cmp(x.name,y.name)

def handle_getScenarios(req):
    data = []
    for scenarioName in manager.GetScenarios():
        if scenarioName == manager.GetRunningScenario():
            running = True
        else:
            running = False
        if scenarioName == manager.GetDefaultScenario():
            active = True
        else:
            active = False

        params = []
        for nodeName in manager.GetScenarioNodes(scenarioName):
            if manager.getScenarioData(scenarioName)['launch_parameters'][nodeName]:
                params.append(NodeLaunchParam(nodeName, manager.getScenarioData(
                    scenarioName)['launch_parameters'][nodeName]))

        data.append(Scenario(scenarioName, manager.GetScenarioNodes(
            scenarioName), active, running, params))
	data.sort(cmp=scenarioCmp)
    return GetScenariosResponse(data)


def handle_getNodesDetails(req):
    nodes = []
    nodes = manager.GetNodesDetails()

    return GetNodesDetailsResponse(nodes)


def handle_generateArduinoLibrary(req):
    return CsOobeHelper.GenerateArduinoLibrary()


def handle_stopRunningScenario(req):
    manager.StopRunningScenario()
    return True


def handle_getRobots(req):
    return GetRobotsResponse(manager.GetRobots())


def handle_getOOBENodes(req):
    return GetOOBENodesResponse(manager.GetOOBENodes())


def handle_runRobot(req):
    print "test"


def handle_getConfigurableNodes(req):
    return GetConfigurableNodesResponse(manager.GetConfigurableNodes())


def handle_runScenario(req):
    return manager.RunScenario(req.scenarioName)


def handle_updateVersion(req):
    return manager.UpdateVersion()


def handle_saveConfiguration(req):
    return manager.SaveConfiguration()


def handle_restartOOBE(req):
    CsOobeHelper.RestartOOBE()




def handle_findLaunchFiles(req):
    return FindLaunchFilesResponse(manager.FindLaunchFiles(req.packageName))


def handle_getLaunchFilePath(req):
    return GetLaunchFilePathResponse(manager.GetLaunchFilePath(req.packageName, req.launchFileName))


def handle_findRunningNodes(req):
    return FindRunningNodesResponse(manager.FindRunningNodes(req.launchFilePath))


def handle_registerNewNode(req):
    return RegisterNewNodeResponse(manager.RegisterNewNode(req.nodeName, req.packageName, req.runningNodeList, req.launchFileName, req.launchParams))


def handle_deleteNode(req):
    ret = manager.DeleteNode(req.nodeName)
    return DeleteNodeResponse(ret, ret == 0)


def handle_editNode(req):
    return EditNodeResponse(manager.EditNode(req.nodeName, req.packageName, req.runningNodeList, req.launchFileName, req.launchParams))


def startROSAPI():
    print "Activating nodes service"
    rospy.Service('/euclid/get_system_nodes',
                  GetSystemNodes, handle_getSystemNodes)
    rospy.Service('/euclid/activate_nodes', ActivateNodes, handle_activateNode)
    rospy.Service('/euclid/deactivate_nodes',
                  DeactivateNodes, handle_deactiveNode)
    rospy.Service('/euclid/stop_running_scenario',
                  StopRunningScenario, handle_stopRunningScenario)
    rospy.Service('/euclid/get_default_scenario',
                  GetDefaultScenario, handle_getDefaultScenario)
    rospy.Service('/euclid/get_params', GetParams, handle_getParams)
    rospy.Service('/euclid/get_configurable_nodes',
                  GetConfigurableNodes, handle_getConfigurableNodes)
    rospy.Service('/euclid/set_param', SetParam, handle_setParam)
    rospy.Service('/euclid/create_scenario',
                  CreateScenario, handle_createScenario)
    rospy.Service('/euclid/restart_system',
                  RestartSystem, handle_restartSystem)
    rospy.Service('/euclid/shutdown_system', Shutdown, handle_shutdownSystem)
    rospy.Service('/euclid/restart_oobe', RestartOOBE, handle_restartOOBE)
    rospy.Service('/euclid/set_default_scenario',
                  SetDefaultScenario, handle_setDefaultScenario)
    rospy.Service('/euclid/remove_scenarios',
                  RemoveScenarios, handle_removeScenarios)
    rospy.Service('/euclid/get_scenarios', GetScenarios, handle_getScenarios)
    rospy.Service('/euclid/generate_arduino_library',
                  GenerateArduinoLibrary, handle_generateArduinoLibrary)
    rospy.Service('/euclid/get_oobe_nodes', GetOOBENodes, handle_getOOBENodes)
    rospy.Service('/euclid/get_robots', GetRobots, handle_getRobots)
    rospy.Service('/euclid/run_scenario', RunScenario, handle_runScenario)
    rospy.Service('/euclid/export_scenarios',
                  ExportScenarios, handle_exportScenarios)
    rospy.Service('/euclid/import_scenarios',
                  ImportScenarios, handle_importScenarios)
    rospy.Service('/euclid/get_xml_data', GetXMLData, handle_getXMLData)
    rospy.Service('/euclid/update_xml_data',
                  UpdateXMLData, handle_updateXMLData)
    rospy.Service('/euclid/find_launch_files',
                  FindLaunchFiles, handle_findLaunchFiles)
    rospy.Service('/euclid/get_launch_file_path',
                  GetLaunchFilePath, handle_getLaunchFilePath)
    rospy.Service('/euclid/find_running_nodes',
                  FindRunningNodes, handle_findRunningNodes)
    rospy.Service('/euclid/register_new_node',
                  RegisterNewNode, handle_registerNewNode)
    rospy.Service('/euclid/update_version',
                  UpdateVersion, handle_updateVersion)
    rospy.Service('/euclid/edit_scenario', EditScenario, handle_editScenario)
    rospy.Service('/euclid/delete_node', DeleteNode, handle_deleteNode)
    rospy.Service('/euclid/edit_node', EditNode, handle_editNode)
    rospy.Service('/euclid/get_nodes_details',
                  GetNodesDetails, handle_getNodesDetails)


if __name__ == '__main__':
    try:

        rospy.init_node('CsConfigurationNode')
        if len(sys.argv) == 2:
            manager = CsConfigModule(sys.argv[1])
        else:
            manager = CsConfigModule("/intel/euclid/config/config.xml")

        startROSAPI()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
