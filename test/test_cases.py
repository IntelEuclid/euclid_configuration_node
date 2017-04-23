
#!/usr/bin/env python

import unittest
import rospy
import subprocess
import time
import os.path
import os
from configuration_node.srv import *


class Utils():
    @staticmethod
    def killSystem():
        print "killing system!"
        subprocess.call(['/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/killSystem.bash'])


class OOBENodesNotEmpty(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_1.xml'])

        oobe_service = rospy.ServiceProxy('/cs/get_oobe_nodes',GetOOBENodes)
        rospy.wait_for_service('/cs/get_oobe_nodes')
        res = oobe_service()
        self.assertTrue(len(res.nodes) == 1)
        self.assertEqual( res.nodes[0], "Test 1 node")
        subprocess.call(['kill','-9',str(proc.pid)])
        
        time.sleep(2)


class OOBENodesEmpty(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_2.xml'])

        oobe_service = rospy.ServiceProxy('/cs/get_oobe_nodes',GetOOBENodes)
        rospy.wait_for_service('/cs/get_oobe_nodes')
        res = oobe_service()
        self.assertTrue(len(res.nodes) == 0)
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(2)


class SystemNodesAll(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_1.xml'])

        oobe_service = rospy.ServiceProxy('/cs/get_system_nodes',GetSystemNodes)
        rospy.wait_for_service('/cs/get_system_nodes')
        res = oobe_service('all')
        self.assertTrue(len(res.nodes) == 1)
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(2)

class SystemNodesActive(unittest.TestCase):
    def runTest(self):
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        Utils.killSystem()
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_1.xml'])

        oobe_service = rospy.ServiceProxy('/cs/get_system_nodes',GetSystemNodes)
        rospy.wait_for_service('/cs/get_system_nodes')
        res = oobe_service('active')
        self.assertTrue(len(res.nodes) == 0)
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(2)


class SystemNodesWrong(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_1.xml'])

        oobe_service = rospy.ServiceProxy('/cs/get_system_nodes',GetSystemNodes)
        rospy.wait_for_service('/cs/get_system_nodes')
        res = oobe_service('koko')
        self.assertTrue(len(res.nodes) == 0)
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(2)

class RunScenarioValid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_1.xml'])

        oobe_service = rospy.ServiceProxy('/cs/run_scenario',RunScenario)
        rospy.wait_for_service('/cs/run_scenario')
        res = oobe_service('Camera')
        self.assertTrue(res.res)
        subprocess.call(['rosnode','kill','/CsScenarioManager'])
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(8)   
        Utils.killSystem()

class RunScenarioNotValid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_1.xml'])
        time.sleep(10)
        oobe_service = rospy.ServiceProxy('/cs/run_scenario',RunScenario)
        rospy.wait_for_service('/cs/run_scenario')
        res = oobe_service('koko')
        self.assertFalse(res.res)
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(2)

class GetActiveScenarioValid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_3.xml'])
        time.sleep(3)
        oobe_service = rospy.ServiceProxy('/cs/get_active_scenario',GetActiveScenario)
        rospy.wait_for_service('/cs/get_active_scenario')
        res = oobe_service()
        self.assertEqual(res.name,'Camera')
        
        time.sleep(2)

class GetActiveScenarioUnvalid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_1.xml'])
        time.sleep(5)
        oobe_service = rospy.ServiceProxy('/cs/get_active_scenario',GetActiveScenario)
        rospy.wait_for_service('/cs/get_active_scenario')
        res = oobe_service()
        self.assertEqual(res.name,'')
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(2)


class SetActiveScenarioValid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        subprocess.call(['cp','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        time.sleep(2)
        oobe_service = rospy.ServiceProxy('/cs/set_active_scenario',SetActiveScenario)
        rospy.wait_for_service('/cs/set_active_scenario')
        res = oobe_service('Camera')
        self.assertTrue(res.res)
        print "Doing the grep.."
        p = subprocess.Popen(['grep', '-A 3','<active_scenario>','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        output, err = p.communicate()
        print "Output: " + output
        expStr="  <active_scenario>\n    <name>Camera</name>\n  </active_scenario>\n  <system_nodes>\n"
        self.assertEqual(output,expStr)
        

class SetActiveScenarioUnvalid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        subprocess.call(['cp','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        time.sleep(2)
        oobe_service = rospy.ServiceProxy('/cs/set_active_scenario',SetActiveScenario)
        rospy.wait_for_service('/cs/set_active_scenario')
        res = oobe_service('koko')
        self.assertFalse(res.res)
        p = subprocess.Popen(['grep', '-A 3','<active_scenario>','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        output, err = p.communicate()
        expStr="  <active_scenario>\n    <name></name>\n  </active_scenario>\n  <system_nodes>\n"
        self.assertEqual(output,expStr)


class GetScenariosTest(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml'])
        time.sleep(2)
        oobe_service = rospy.ServiceProxy('/cs/get_scenarios',GetScenarios)
        rospy.wait_for_service('/cs/get_scenarios')
        res = oobe_service()

        self.assertEqual(len(res.scenarios),1)
        
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(1)

class GetRobotsTest(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml'])
        time.sleep(5)
        oobe_service = rospy.ServiceProxy('/cs/get_robots',GetRobots)
        rospy.wait_for_service('/cs/get_robots')
        res = oobe_service()

        self.assertEqual(len(res.robots),1)
        
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(1)

class RemoveScenarioValid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        subprocess.call(['cp','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        time.sleep(5)
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        time.sleep(2)
        oobe_service = rospy.ServiceProxy('/cs/remove_scenarios',RemoveScenarios)
        rospy.wait_for_service('/cs/remove_scenarios')
        res = oobe_service(['Camera'])
        self.assertTrue(res.res)
        p = subprocess.Popen(['grep','-A 1','<scenarios>','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        output, err = p.communicate()
        print "output: " + output
        expStr="  <scenarios>\n    </scenarios>\n"
        self.assertEqual(output,expStr)
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(1)  

class RemoveScenarioUnvalid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        subprocess.call(['cp','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        time.sleep(5)
        oobe_service = rospy.ServiceProxy('/cs/remove_scenarios',RemoveScenarios)
        rospy.wait_for_service('/cs/remove_scenarios')
        res = oobe_service(['koko'])
        self.assertFalse(res.res)
       
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(1)  


class CreateScenarioIllegal(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        # subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml'])
        time.sleep(2)
        oobe_service = rospy.ServiceProxy('/cs/create_scenario',CreateScenario)
        rospy.wait_for_service('/cs/create_scenario')
        res = oobe_service('koko space',['Test 1 node'])
        self.assertFalse(res.res)
        res = oobe_service('koko$dollar',['Test 1 node'])
        self.assertFalse(res.res)
        res = oobe_service('Camera',['Test 1 node'])
        self.assertFalse(res.res)
        res = oobe_service('Valid',[])
        self.assertFalse(res.res)
        res = oobe_service('Valid',['WhoAmI?'])
        self.assertFalse(res.res)
       
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(1)  

class CreateScenarioValid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        # subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        subprocess.call(['cp','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml','/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_t.xml'])
        time.sleep(2)
        oobe_service = rospy.ServiceProxy('/cs/create_scenario',CreateScenario)
        rospy.wait_for_service('/cs/create_scenario')
        res = oobe_service('TestScenario',['Test 1 node'])
        self.assertTrue(res.res)
        #rospy.wait_for_service('/cs/save_configuration')
        time.sleep(3)
        subprocess.call(['rosnode','kill','/CsScenarioManager'])
        time.sleep(1)
        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(1)  

class GenerateArduinoTest(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        # subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_4.xml'])
        time.sleep(2)
        oobe_service = rospy.ServiceProxy('/cs/generate_arduino_library',GenerateArduinoLibrary)
        rospy.wait_for_service('/cs/generate_arduino_library')
        subprocess.call(['rm','-rf','/intel/euclid/public_html/files/euclid_lib.zip'])

        res = oobe_service()
        self.assertTrue(res.res)
        self.assertTrue(os.path.isfile('/intel/euclid/public_html/files/euclid_lib.zip'))

        subprocess.call(['kill','-9',str(proc.pid)])
        time.sleep(1)  


class BadConfigFile(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        # subprocess.call(['rosnode','kill','/CsConfigurationNode'])
        proc = subprocess.Popen(['rosrun','configuration_node','CsConfigModule.py' ,'/intel/euclid/euclid_ws/src/system_nodes/configuration_node/test/test_config_bad.xml'])
        time.sleep(2)
        rospy.wait_for_service('/cs/generate_arduino_library')
        time.sleep(2)
        subprocess.call(['rosnode','kill','/CsDeviceMonitor'])
        subprocess.call(['rosnode','kill','/CsNetworkManager'])
        
        self.assertTrue(True)


class ScenarioManagerError(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #ret = subprocess.call(['ps auwwxf | grep CsScenarioManager | grep -v grep'],shell=True)
        proc = subprocess.Popen(['rosrun','configuration_node','CsScenarioManager.py' ,'scenario:=koko'])
        time.sleep(5)
        ret = subprocess.call(['ps auwwxf | grep CsScenarioManager | grep -v grep'],shell=True)
        self.assertEqual(ret,1)
       
class ScenarioManagerValid(unittest.TestCase):
    def runTest(self):
        Utils.killSystem()
        #ret = subprocess.call(['ps auwwxf | grep CsScenarioManager | grep -v grep'],shell=True)
        proc = subprocess.Popen(['rosrun','configuration_node','CsScenarioManager.py' ,'Camera'])
        time.sleep(5)
        ret = subprocess.call(['ps auwwxf | grep CsScenarioManager | grep -v grep'],shell=True)
        self.assertEqual(ret,0)
        
        Utils.killSystem()


class RunTests(unittest.TestSuite):
    def __init__(self):
        super(RunTests,self).__init__()
        self.addTest(OOBENodesNotEmpty())
        self.addTest(OOBENodesEmpty())
        self.addTest(SystemNodesAll())
        self.addTest(SystemNodesActive())
        self.addTest(SystemNodesWrong())
        self.addTest(RunScenarioValid())
        self.addTest(RunScenarioNotValid())
        self.addTest(GetActiveScenarioValid())
        self.addTest(GetActiveScenarioUnvalid())
        self.addTest(SetActiveScenarioValid())
        self.addTest(SetActiveScenarioUnvalid())
        self.addTest(GetScenariosTest())
        self.addTest(GetRobotsTest())
        self.addTest(RemoveScenarioValid())
        self.addTest(RemoveScenarioUnvalid())
        self.addTest(CreateScenarioIllegal())
        self.addTest(CreateScenarioValid())
        self.addTest(GenerateArduinoTest())
        self.addTest(BadConfigFile())
        self.addTest(ScenarioManagerError())
        self.addTest(ScenarioManagerValid())




