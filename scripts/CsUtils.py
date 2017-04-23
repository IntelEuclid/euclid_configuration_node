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


import rosnode
import subprocess
import ast
from dynamic_reconfigure import client
from rospy.timer import sleep
import os

def IsNodeRunning(nodeName):
    return rosnode.rosnode_ping(nodeName, 1, False)

def KillNode(nodeName):
    subprocess.Popen(['rosnode', 'kill', nodeName])

#TODO, handle timeout exception if needed ..
def LaunchNode(packageName,launchFile,nodesList,timeOut=10):
    print "launching: " + launchFile
    my_env = os.environ.copy()
    my_env["TURTLEBOT_3D_SENSOR"] = "commonsense"
    my_env["TURTLEBOT_STACKS"]="hexagons"
    proc = subprocess.Popen(['roslaunch', packageName, launchFile],env=my_env)
    allNodes = 0
    while allNodes != len(nodesList) and timeOut >0:
        allNodes = 0
        sleep(1)
        for node in nodesList:
            if rosnode.rosnode_ping(node,1,False) == True:
                allNodes = allNodes + 1
            else:
                break
        timeOut = timeOut -1
    print "Process pid: " + str(proc.pid)
    return proc.pid
        
#TODO: check refactoring      
#TODO: timeout should be "#defined"
def GetConfigurationYamlFile(folderName,packageName,nodeName,fileName):
    outputFile = fileName
    retVal = 0
    retVal = subprocess.call(["rosrun", "dynamic_reconfigure", "dynparam", "dump",nodeName, outputFile + ".tmp","-t 3"])
    subprocess.call(['/intel/euclid/oobe-utils/configuration/fixYaml.bash',outputFile + ".tmp", outputFile])
    return retVal == 0


def LoadConfiguration(nodeName,fileName):
    outputFile = fileName
    print "Got: " + nodeName + ", " + fileName
    retVal = subprocess.call(["rosrun", "dynamic_reconfigure", "dynparam", "load",nodeName, outputFile,"-t 5"])
    return retVal == 0

def SetParam(nodeName,param,value):
    proc = subprocess.Popen(["rosrun", "dynamic_reconfigure", "dynparam", "set",nodeName, param,value,"-t 3"],stdout=subprocess.PIPE)
    for line in proc.stdout:
        if line.find("couldn't set parameters") != -1:
            return False
    return True

def GetParams(nodeName):
    proc = subprocess.Popen(['rosrun','dynamic_reconfigure','dynparam','get',nodeName],stdout=subprocess.PIPE)
    for data in  proc.stdout:
        dic = ast.literal_eval(data)
        #print dic
    return dic

# def Reboot():
#     subprocess.call(['reboot'])

# def Shutdown():
#     subprocess.call(['shutdown','-P','0'])

# def RestartOOBE():
#     subprocess.call(['service','oobe-init','restart-oobe'])
        
# def GenerateArduinoLibrary():
#     retVal = subprocess.call(['sh','/intel/euclid/oobe-utils/generateArduinoLibrary/generateArduinoLibrary.sh'])
#     return retVal == 0   

# def exportSettings():
#     retVal = subprocess.call(['sh','/intel/euclid/oobe-utils/exportImportSettings/exportSettings.sh'])
#     return retVal == 0       
    

if __name__ == '__main__':
    print IsNodeRunning('/RealsenseNodelet')
