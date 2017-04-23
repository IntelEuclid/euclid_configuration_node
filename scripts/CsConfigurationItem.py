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

import xml.etree.ElementTree as ET
from xml.dom import minidom
import CsUtils
class ConfigurationItem:
    _configurationFolder = "/intel/euclid/oobe-utils/configuration"
    def __init__(self,configurationFile):
        self._configurationFile = configurationFile
        self.parseConfig()
        
    def parseConfig(self):
        tree = ET.parse(self._configurationFile)
        root = tree.getroot()
     
        self.configuration_name = root.findall("configuration_name")[0].text 
        self.package = root.findall("package")[0].text 
        self.launch_file = root.findall("launch_file")[0].text 
        

    @staticmethod   
    def Load(configurationItemFileName):
        print "Loading config: " + configurationItemFileName
        tree = ET.parse(configurationItemFileName)
        root = tree.getroot()
     
        configuration_name = root.findall("configuration_name")[0]
        nodes  = root.findall('nodes')
        for node in nodes[0]:
            for att in node:
                if att.tag == 'yaml':
                    if att.text is None:
                        print "attr text: none"
                    else:
                        print "attr text: " + att.text
                        CsUtils.LoadConfiguration(node.get('name'),att.text)
    
    @staticmethod
    def CreateNewConfigurationItem(configurationName,packageName,nodesList):
        print "new configuration item object"
        config = ET.Element("configuration_item")
        ET.SubElement(config,"configuration_name").text=configurationName
        #ET.SubElement(config,"nodeName").text=nodeName
        ET.SubElement(config, "package").text = packageName

        nodes = ET.SubElement(config,"nodes")
        first = True
        for node in nodesList:
            nodeElement = ET.SubElement(nodes,"node",name=node)
            
            outputFile = ConfigurationItem._configurationFolder + "/" + configurationName + "." + packageName + "."+node + ".yaml";
            if  CsUtils.GetConfigurationYamlFile(ConfigurationItem._configurationFolder,packageName,node,outputFile) == True:
                print "Setting file: " +  outputFile
                ET.SubElement(nodeElement, "yaml").text = outputFile  
            else:
                print "Problem getting yaml file for node: " + node
                ET.SubElement(nodeElement, "yaml")
        xmlstr = minidom.parseString(ET.tostring(config)).toprettyxml(indent="  ")
        outputFile = ConfigurationItem._configurationFolder + "/" + configurationName + "." + packageName + ".cscfg";
        with open(outputFile,"w") as f:
            f.write(xmlstr)
        f.close()
       # return ConfigurationItem(outputFile)
        #etree.parse(etree.tostring(tree,pretty_print=True))
        #tree.write(ConfigurationItem._configurationFolder + "/" + packageName + ".cscfg",xml_declaration=True)
        
    @staticmethod
    def init(configurationFolder):
        print "Using: " + configurationFolder
        ConfigurationItem._configurationFolder = configurationFolder

if __name__ == '__main__':
    ConfigurationItem.init("/intel/euclid/oobe-utils/configuration")
    #ConfigurationItem.CreateNewConfigurationItem("majd","realsense_camera",['RealsenseNodelet'])

    #ConfigurationItem.Load('/home/ros/oobe-libs/configuration/R200.realsense_camera.cscfg')

    
    
    
