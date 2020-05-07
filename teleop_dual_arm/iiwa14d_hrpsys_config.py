#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP

class iiwa14dRobotHrpsysConfigurator(HrpsysConfigurator):
    def getRTCList (self):
        return self.getRTCListUnstable()
    def init (self, robotname="iiwa14d", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        print "initialize rtc parameters"
        #self.setStAbcParameters()
        self.setResetPose(1)
        time.sleep(2)
        self.wbms_svc.startWholeBodyMasterSlave()

    def getRTCListUnstable(self):
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            # ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            # ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            # ['octd', "ObjectContactTurnaroundDetector"],
            # ['es', "EmergencyStopper"],
            ['wbms', "WholeBodyMasterSlave"],
            ['rfu', "ReferenceForceUpdater"],
            ['ic', "ImpedanceController"],
            # ['abc', "AutoBalancer"],
            # ['st', "Stabilizer"],
            #['co', "CollisionDetector"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            # ['tl', "ThermoLimiter"],
            # ['bp', "Beeper"],
            # ['acf', "AccelerationFilter"],
            ['log', "DataLogger"]
            ]
        
    def defJointGroups (self):
        # head_group = ['head', []]
        # rarm_group = ['rarm', ['RARM_SHOULDER_P', 'RARM_SHOULDER_R', 'RARM_SHOULDER_Y', 'RARM_ELBOW', 'RARM_WRIST_Y', 'RARM_WRIST_P', 'RARM_WRIST_R']]
        larm_group = ['larm', ['iiwa_larm_joint_1', 'iiwa_larm_joint_2', 'iiwa_larm_joint_3', 'iiwa_larm_joint_4', 'iiwa_larm_joint_5', 'iiwa_larm_joint_6', 'iiwa_larm_joint_7']]
        rarm_group = ['rarm', ['iiwa_rarm_joint_1', 'iiwa_rarm_joint_2', 'iiwa_rarm_joint_3', 'iiwa_rarm_joint_4', 'iiwa_rarm_joint_5', 'iiwa_rarm_joint_6', 'iiwa_rarm_joint_7']]
        self.Groups = [rarm_group, larm_group]

    #def setStAbcParameters (self):
        # not in use


    def iiwa14dInitPose (self):
        return [0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0]

    def iiwa14dResetPose (self):
        return [0,  0, 0, 0, 1.5708, 1.5708, 0, 0.7854, 0, 0,  0, 0, 0, 1.5708, -1.5708, 0, -0.7854, 0, 0]

    def iiwa14dTableTopPose (self):
        return [0,  0, 0, 0, 1.5708, 1.5708, -1.5708, 1.5708, -0.7854, 0,   0, 0, 0, 1.5708, -1.5708, -1.5708, 1.5708, 0.7854, 0]

    def setInitPose(self, tm=5):
        self.seq_svc.setJointAngles(self.iiwa14dInitPose(), tm)

    def setResetPose(self, tm=5):
        # self.seq_svc.setJointAngles(self.iiwa14dResetPose(), tm)
        self.seq_svc.setJointAngles(self.iiwa14dTableTopPose(), tm)


    def __init__(self, robotname=""):
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()

if __name__ == '__main__':
    hcf = iiwa14dRobotHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
