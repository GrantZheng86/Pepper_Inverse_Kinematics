from scipy.spatial.transform import Rotation
import numpy as np
from abc import ABC, abstractclassmethod
import mpl_toolkits.mplot3d as plt3d
from pip._internal import self_outdated_check


class Pepper:
    def __init__(self):
        
        
        # The following transform location number and quaternion are
        # based on the zero position of pepper. They will be later
        # updated when the robot starts to move
        
        # For details go to the website
        # http://doc.aldebaran.com/2-5/family/pepper_technical/links_pep.html
        base2shoulderPitch = {
            'L': Transform([-0.057, 0.14974, 0.08682], [0., 0., 0., 1.]),
            'R': Transform([-0.057, -0.14974, 0.08682], [0., 0., 0., 1.])
        }
        
        shoulderPitch2shoulderRoll = {
            'L': Transform([0., 0., 0.], [0., 0., 0., 1.]),
            'R': Transform([0., 0., 0.], [0., 0., 0., 1.])
            }

        shoulderRoll2elbowYaw = {
            'L': Transform([0.1812, 0.015, 0.00013], [0., 0., 0., 1.]),
            'R': Transform([0.1812, -0.015, 0.00013], [0., 0., 0., 1.])
        }
        
        elbowYaw2elbowRoll = {
            'L': Transform([0., 0., 0.], [0., 0., 0., 1.]),
            'R': Transform([0., 0., 0.], [0., 0., 0., 1.])
            }
        
        # In zero position, pepper forearm are poining upwards by 9 degrees
        elbowRoll2wristYaw = {
            'L': Transform([0.15, 23.6e-3, 22.84e-3], [0, -0.0784593, 0, 0.9969173]),
            'R': Transform([0.15, 23.6e-3, 22.84e-3], [0, 0.0784593, 0, 0.9969173])
        }

        wristYaw2hand = {
            'L': Transform([0.0695, 0.0, -0.03030], [0., 0., 0., 1.]),
            'R': Transform([0.0695, 0.0, -0.03030], [0., 0., 0., 1.])
        }
        
        # JointWithTransforms(string jointName, JointWithTransforms nextJoint,
        #                     Transform transformToNext, Array[3] cartisianLocation)
        self.baseL = JointWithTransforms('baseL', None, base2shoulderPitch['L'], 
                                        location=[0., 0., 0.])
        self.baseR = JointWithTransforms('baseR', None, base2shoulderPitch['R'],
                                        location=[0., 0., 0.])

        self.shoulderPitchL = JointWithTransforms('shoulderPitchL', None, 
                                                  shoulderPitch2shoulderRoll['L'],
                                                  location=None)
        self.shoulderPitchR = JointWithTransforms('shoulderPitchR', None, 
                                                  shoulderPitch2shoulderRoll['R'],
                                                  location=None)

        self.baseL.toJoint = self.shoulderPitchL
        self.baseR.toJoint = self.shoulderPitchR
        
        self.shoulderRollL = JointWithTransforms('shoulderRollL', None,
                                                 shoulderRoll2elbowYaw['L'],
                                                 location = None)
        self.shoulderRollR = JointWithTransforms('shoulderRollR', None,
                                                 shoulderRoll2elbowYaw['R'],
                                                 location = None)
        
        self.shoulderPitchL.toJoint = self.shoulderRollL
        self.shoulderPitchR.toJoint = self.shoulderRollR
        

        self.elbowYawL = JointWithTransforms('elbowYawL', None,
                                             elbowYaw2elbowRoll['L'],
                                             location=None)
        self.elbowYawR = JointWithTransforms('elbowYawR', None, 
                                             elbowYaw2elbowRoll['R'],
                                             location=None)

        self.shoulderRollL.toJoint = self.elbowYawL
        self.shoulderRollR.toJoint = self.elbowYawR

        self.elbowRollL = JointWithTransforms('elbowRollL', None,
                                              elbowRoll2wristYaw['L'],
                                              location = None)
        self.elbowRollR = JointWithTransforms('elbowRollR', None,
                                              elbowRoll2wristYaw['R'],
                                              location = None)
        
        self.elbowYawL.toJoint = self.elbowRollL
        self.elbowYawR.toJoint = self.elbowRollR
        
        
        self.wristYawL = JointWithTransforms('wristYawL', None,
                                             wristYaw2hand['L'],
                                             location=None)
        self.wristYawR = JointWithTransforms('wristYawR', None, 
                                             wristYaw2hand['R'],
                                             location=None)

        self.elbowYawL.toJoint = self.wristYawL
        self.elbowYawR.toJoint = self.wristYawR

        self.handL = JointWithTransforms('handL', None, None, location=None)
        self.handR = JointWithTransforms('handR', None, None, location=None)

        self.wristYawL.toJoint = self.handL
        self.wristYawR.toJoint = self.handR

        #self.updateJointLocation()
        self._initJointLocations()

    def _initJointLocations(self):
        self.shoulderPitchL.location = np.add(self.baseL.location, self.baseL.transform.translation)
        self.shoulderPitchR.location = np.add(self.baseR.location, self.baseR.transform.translation)
        self.shoulderRollL.location  = np.add(self.shoulderPitchL.location, self.shoulderPitchL.transform.translation)
        self.shoulderRollR.location  = np.add(self.shoulderPitchR.location, self.shoulderPitchR.transform.translation)
        
        self.elbowYawL.location      = np.add(self.shoulderRollL.location, self.shoulderRollL.transform.translation)
        self.elbowYawR.location      = np.add(self.shoulderRollR.location, self.shoulderRollR.transform.translation)
        self.elbowRollL.location     = np.add(self.elbowYawL.location, self.elbowYawL.transform.translation)
        self.elbowRollR.location     = np.add(self.elbowYawR.location, self.elbowYawR.transform.translation)
        
        self.wristYawL.location      = np.add(self.elbowRollL.location, self.elbowRollL.transform.translation)
        self.wristYawR.location      = np.add(self.elbowRollR.location, self.elbowRollR.transform.translation)
        
        self.handL.location          = np.add(self.wristYawL.location, self.wristYawL.transform.translation)
        self.handR.location          = np.add(self.wristYawR.location, self.wristYawR.transform.translation)
        
        
    def updateJointLocation(self):
        
        start = np.eye(4)
        currentJoint = self.baseL
        
        while currentJoint.toJoint is not None:
            start = np.matmul(start, currentJoint.transform.toHomoMatrix())
            currentJoint.toJoint.location = start[0:3,3]
            currentJoint = currentJoint.toJoint
            
        currentJoint = self.baseR
        
        while currentJoint.toJoint is not None:
            start = np.matmul(start, currentJoint.transform.toHomoMatrix())
            currentJoint.toJoint.location = start[0:3,3]
            currentJoint = currentJoint.toJoint
            
            
            
            
        
        
        
        

    def getArmLength(self, segment):

        if segment == 1:
            return np.sqrt(np.sum(np.square(self.baseL.transform.translation)))
        elif segment == 2:
            return np.sqrt(np.sum(np.square(self.shoulderL.transform.translation)))
        elif segment == 3:
            return np.sqrt(np.sum(np.square(self.elbowL.transform.translation)))
        elif segment == 4:
            return np.sqrt(np.sum(np.square(self.wrist.transform.translation)))
        else:
            raise Exception("Invalid index entered, only from 1 to 4")

    def get3DLine(self, side):
        xToReturn = []
        yToReturn = []
        zToReturn = []

        if side == "L":
            baseShoulderX = (self.baseL.location[0], self.shoulderPitchL.location[0])
            baseShoulderY = (self.baseL.location[1], self.shoulderPitchL.location[1])
            baseShoulderZ = (self.baseL.location[2], self.shoulderPitchL.location[2])
            xToReturn.append(baseShoulderX[0])
            xToReturn.append(baseShoulderX[1])
            yToReturn.append(baseShoulderY[0])
            yToReturn.append(baseShoulderY[1])
            zToReturn.append(baseShoulderZ[0])
            zToReturn.append(baseShoulderZ[1])

            shoulderElbowX = (self.shoulderRollL.location[0], self.elbowYawL.location[0])
            shoulderElbowY = (self.shoulderRollL.location[1], self.elbowYawL.location[1])
            shoulderElbowZ = (self.shoulderRollL.location[2], self.elbowYawL.location[2])
            shoulderElbow = plt3d.art3d.Line3D(shoulderElbowX, shoulderElbowY, shoulderElbowZ)
            xToReturn.append(shoulderElbowX[0])
            xToReturn.append(shoulderElbowX[1])
            yToReturn.append(shoulderElbowY[0])
            yToReturn.append(shoulderElbowY[1])
            zToReturn.append(shoulderElbowZ[0])
            zToReturn.append(shoulderElbowZ[1])
            
            elbowWristX = (self.elbowRollL.location[0], self.wristYawL.location[0])
            elbowWristY = (self.elbowRollL.location[1], self.wristYawL.location[1])
            elbowWristZ = (self.elbowRollL.location[2], self.wristYawL.location[2])
            elbowWrist = plt3d.art3d.Line3D(elbowWristX, elbowWristY, elbowWristZ)
            xToReturn.append(elbowWristX[0])
            xToReturn.append(elbowWristX[1])
            yToReturn.append(elbowWristY[0])
            yToReturn.append(elbowWristY[1])
            zToReturn.append(elbowWristZ[0])
            zToReturn.append(elbowWristZ[1])
            
            wristHandX = (self.wristYawL.location[0], self.handL.location[0])
            wristHandY = (self.wristYawL.location[1], self.handL.location[1])
            wristHandZ = (self.wristYawL.location[2], self.handL.location[2])
            wristHand = plt3d.art3d.Line3D(wristHandX, wristHandY, wristHandZ)
            xToReturn.append(wristHandX[0])
            xToReturn.append(wristHandX[1])
            yToReturn.append(wristHandY[0])
            yToReturn.append(wristHandY[1])
            zToReturn.append(wristHandZ[0])
            zToReturn.append(wristHandZ[1])
        
        elif side == "R":
            baseShoulderX = (self.baseR.location[0], self.shoulderPitchR.location[0])
            baseShoulderY = (self.baseR.location[1], self.shoulderPitchR.location[1])
            baseShoulderZ = (self.baseR.location[2], self.shoulderPitchR.location[2])
            baseShoulder = plt3d.art3d.Line3D(baseShoulderX, baseShoulderY, baseShoulderZ)
            xToReturn.append(baseShoulderX[0])
            xToReturn.append(baseShoulderX[1])
            yToReturn.append(baseShoulderY[0])
            yToReturn.append(baseShoulderY[1])
            zToReturn.append(baseShoulderZ[0])
            zToReturn.append(baseShoulderZ[1])
            
            shoulderElbowX = (self.shoulderRollR.location[0], self.elbowYawR.location[0])
            shoulderElbowY = (self.shoulderRollR.location[1], self.elbowYawR.location[1])
            shoulderElbowZ = (self.shoulderRollR.location[2], self.elbowYawR.location[2])
            #shoulderElbow = plt3d.art3d.Line3D(shoulderElbowX, shoulderElbowY, shoulderElbowZ)
            xToReturn.append(shoulderElbowX[0])
            xToReturn.append(shoulderElbowX[1])
            yToReturn.append(shoulderElbowY[0])
            yToReturn.append(shoulderElbowY[1])
            zToReturn.append(shoulderElbowZ[0])
            zToReturn.append(shoulderElbowZ[1])
            
            elbowWristX = (self.elbowRollR.location[0], self.wristYawR.location[0])
            elbowWristY = (self.elbowRollR.location[1], self.wristYawR.location[1])
            elbowWristZ = (self.elbowRollR.location[2], self.wristYawR.location[2])
            #elbowWrist = plt3d.art3d.Line3D(elbowWristX, elbowWristY, elbowWristZ)
            xToReturn.append(elbowWristX[0])
            xToReturn.append(elbowWristX[1])
            yToReturn.append(elbowWristY[0])
            yToReturn.append(elbowWristY[1])
            zToReturn.append(elbowWristZ[0])
            zToReturn.append(elbowWristZ[1])
            
            wristHandX = (self.wristYawR.location[0], self.handR.location[0])
            wristHandY = (self.wristYawR.location[1], self.handR.location[1])
            wristHandZ = (self.wristYawR.location[2], self.handR.location[2])
            #wristHand = plt3d.art3d.Line3D(wristHandX, wristHandY, wristHandZ)
            xToReturn.append(wristHandX[0])
            xToReturn.append(wristHandX[1])
            yToReturn.append(wristHandY[0])
            yToReturn.append(wristHandY[1])
            zToReturn.append(wristHandZ[0])
            zToReturn.append(wristHandZ[1])
            
        
        return xToReturn, yToReturn, zToReturn


class QuatAndTransform(ABC):

    def __init__(self, translation, quaternion):
        self.translation = translation
        self.quaternion = quaternion
        super().__init__()


class JointWithTransforms:

    # TODO: Add position and orientation information to this class
    def __init__(self, name, toJoint, transform, location):
        self.name = name
        self.toJoint = toJoint
        self.transform = transform
        self.location = location

    def updateTransforms(self, xAngle, yAngle, zAngle):
        temp_r = Rotation.from_euler("xyz", [xAngle, yAngle, zAngle])
        self.transform.quaternion = temp_r.as_quat()
        


class Transform(QuatAndTransform):

    def __init__(self, translation, rotation):
        assert len(translation) == 3, "translation must contain 3 elements"
        assert len(rotation) == 4, "rotation quaternion must contain 4 elements"
        super().__init__(translation, rotation)

    def toRotationMatrix(self):
        toReturn = Rotation.from_quat(self.quaternion)
        toReturn = toReturn.as_matrix()
        return toReturn

    def toHomoMatrix(self):
        rotMtx = self.toRotationMatrix()
        toReturn = np.eye(4)
        toReturn[0:3, 0:3] = rotMtx
        toReturn[0:3, 3] = self.translation
        return toReturn


class LocationAndOrientation(QuatAndTransform):

    def __init__(self, translation, rotation):
        assert len(translation) == 3, "translation must contain 3 elements"
        assert len(rotation) == 4, "rotation quaternion must contain 4 elements"
        super().__init__(translation, rotation)
