#!/usr/bin/env python

import rospy
from robotnik_msgs.msg import inputs_outputs
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
import fileinput

class CalibrationNode:
    def __init__(self):
        self.values = []
        self.mean = []
        self.start_calibration = False
        self.calibration_duration = 1
        self.n = 0
        self.initial_calibration = False
        self.kill_node = True
        self.len_unknown = True
        self.init_time = rospy.get_rostime()
        self.file_name = 'robot_params (copy).env'
        self.file_path = '/home/robot/robot_params/'
        self.env_var = 'ROBOT_JOINT_POTENTIOMETER_VOLTAGE_CALIBRATION'
        self.topic_sub = '/robot/robotnik_base_hw/io'
        self.subscriber = rospy.Subscriber(self.topic_sub, inputs_outputs, self.ioCallback)
        self.service = rospy.Service('calibrate_wheels', Trigger, self.calibrationSrv)
    
    def getParams(self):
        self.file_name = rospy.get_param('file_name', self.file_name, self.file_name)
        self.file_path = rospy.get_param('file_path', self.file_path, self.file_path)
        self.env_var = rospy.get_param('env_var', self.env_var, self.env_var)
        self.topic_sub = rospy.get_param('topic_sub', self.topic_sub, self.topic_sub)
        self.calibration_duration = rospy.get_param('calibration_duration', self.calibration_duration, self.calibration_duration)
        self.initial_calibration = rospy.get_param('initial_calibration', self.initial_calibration, self.initial_calibration)
        self.kill_node = rospy.get_param('kill_node', self.kill_node, self.kill_node)

    def ioCallback(self, data):
        if self.len_unknown:
            self.n = len(data.analog_inputs) - 4
            self.len_unknown = False

        if self.start_calibration == True:
            self.values.append(list(data.analog_inputs))
    
    def calibrationSrv(self, data):
        self.start_calibration = True
        self.values = []
        response = TriggerResponse()
        response.success = True
        response.message = 'Starting wheel calibration'
        self.init_time = rospy.get_rostime()
        return response

    def computeMean(self, data):
        for i in range(self.n):
            acc_sum = 0
            for j in range(len(data)):
                acc_sum = acc_sum + data[j][i+4]
            self.mean.append(acc_sum/len(data))
    
    def calibrate(self):
        if self.start_calibration or self.initial_calibration:
            if rospy.get_rostime() - self.init_time > rospy.Duration(self.calibration_duration):
                self.start_calibration = False
                self.initial_calibration = False
                self.computeMean(self.values)
                rospy.loginfo('Mean values obtained. Writing in file ' + self.file_name)
                self.storeValues()

    def createLine(self):
        line = 'export ' + self.env_var + '=[1.0,1.0,1.0,1.0,'
        for i in range(self.n):
            if i < self.n-1:
                line = line + str(self.mean[i]) + ','
            else:
                line = line + str(self.mean[i]) + ']'
        return line

    def storeValues(self):
        target_line = 'export ' + self.env_var
        for line in fileinput.FileInput(self.file_path + self.file_name, inplace=True):
            if line.startswith(target_line):
                line = self.createLine()
                print(line, end='\n')
                self.store_results = False
            else:
                print(line, end='')
        fileinput.close()
        rospy.loginfo('File ' + self.file_name + ' modified. New values for ' + self.env_var + ': ' + str(self.mean))

def main():
    rospy.init_node('calibration_node')
    calibration_node = CalibrationNode()
    rate = rospy.Rate(10)
    calibration_node.calibrate()
    while not rospy.is_shutdown() and not calibration_node.kill_node:
        calibration_node.calibrate()
        rate.sleep()

if __name__ == "__main__":
    main()
