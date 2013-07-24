import xml.etree.cElementTree as ET
from LoadRobotConfiguration import LoadRobotConfiguration
import Communication
from Communication import CommSerial
import Actuator

comm_tty = CommSerial()  #TODO read a configuration file to use the correct parameters for CommSimulator
comm_tty.connect()
actuator_tty = Actuator.Actuator(comm_tty)

root = ET.Element("root")
lucy = ET.SubElement(root, "Lucy")

config = LoadRobotConfiguration()
frameIt = 0
ri = raw_input('presione \'c\' para capturar otra tecla para terminar \n')
while(ri == 'c'):
    frame = ET.SubElement(lucy, "frame")
    frame.set("number" , str(frameIt))
    for joint in config.getJointsName():
        xmlJoint = ET.SubElement(frame, joint)
        joint_id = config.loadJointId(joint)
        print joint_id
        pos = actuator_tty.get_position(joint_id)
        #actuator_tty.move_actuator(frameIt,500,700)
        #print pos
        #xmlJointAngle = xmlJoint.set("angle" , "3")
        xmlJointAngle = xmlJoint.set("angle" , str(pos))
    ri = raw_input('presione \'c\' para capturar otra tecla para terminar \n')
    frameIt = frameIt + 1

tree = ET.ElementTree(root)
tree.write("poses.xml")