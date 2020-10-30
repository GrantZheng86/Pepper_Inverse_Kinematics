from scipy.spatial.transform import Rotation
import numpy as np
from abc import ABC, abstractclassmethod
import mpl_toolkits.mplot3d as plt3d


class Pepper:
    def __init__(self):
        baseToShoulder = {
            'L': Transform([-0.057, 0.14974, 0.08682], [0., 0., 0., 1.]),
            'R': Transform([-0.057, -0.14974, 0.08682], [0., 0., 0., 1.])
        }

        shoulderToElbow = {
            'L': Transform([0.1812, 0.015, 0.00013], [0., 0., 0., 1.]),
            'R': Transform([0.1812, -0.015, 0.00013], [0., 0., 0., 1.])
        }

        elbowToWrist = {
            'L': Transform([0.15, 23.6e-3, 22.84e-3], [0, -0.0784593, 0, 0.9969173]),
            'R': Transform([0.15, 23.6e-3, 22.84e-3], [0, 0.0784593, 0, 0.9969173])
        }

        wristToHand = {
            'L': Transform([0.0695, 0.0, -0.03030], [0., 0., 0., 1.]),
            'R': Transform([0.0695, 0.0, -0.03030], [0., 0., 0., 1.])
        }

        self.baseL = JointWithTransforms('baseL', None, baseToShoulder['L'], location=[0., 0., 0.])
        self.baseR = JointWithTransforms('baseR', None, baseToShoulder['R'], location=[0., 0., 0.])

        self.shoulderL = JointWithTransforms('shoulderL', None, shoulderToElbow['L'],
                                             location=None)
        self.shoulderR = JointWithTransforms('shoulderR', None, shoulderToElbow['R'],
                                             location=None)

        self.baseL.toJoint = self.shoulderL
        self.baseR.toJoint = self.shoulderR

        self.elbowL = JointWithTransforms('elbowL', None, elbowToWrist['L'],
                                          location=None)
        self.elbowR = JointWithTransforms('elbowR', None, elbowToWrist['R'],
                                          location=None)

        self.shoulderL.toJoint = self.elbowL
        self.shoulderR.toJoint = self.elbowR

        self.wristL = JointWithTransforms('wristL', None, wristToHand['L'],
                                          location=None)
        self.wristR = JointWithTransforms('wristR', None, wristToHand['R'],
                                          location=None)

        self.elbowL.toJoint = self.wristL
        self.elbowR.toJoint = self.wristR

        self.handL = JointWithTransforms('handL', None, None, location=None)
        self.handR = JointWithTransforms('handR', None, None, location=None)

        self.wristL.toJoint = self.handL
        self.wristR.toJoint = self.handR

        self._initJointLocations()

    def _initJointLocations(self):
        self.shoulderL.location = np.add(self.baseL.location, self.baseL.transform.translation)
        self.shoulderR.location = np.add(self.baseR.location, self.baseR.transform.translation)
        self.elbowL.location = np.add(self.shoulderL.location, self.shoulderL.transform.translation)
        self.elbowR.location = np.add(self.shoulderR.location, self.shoulderR.transform.translation)
        self.wristL.location = np.add(self.elbowL.location, self.elbowL.transform.translation)
        self.wristR.location = np.add(self.elbowR.location, self.elbowR.transform.translation)
        self.handL.location = np.add(self.wristL.location, self.wristL.transform.translation)
        self.handR.location = np.add(self.wristR.location, self.wristR.transform.translation)

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

        if side == "L":
            baseShoulderX = (self.baseL.location[0], self.shoulderL.location[0])
            baseShoulderY = (self.baseL.location[1], self.shoulderL.location[1])
            baseShoulderZ = (self.baseL.location[2], self.shoulderL.location[2])
            baseShoulder = plt3d.art3d.Line3D(baseShoulderX, baseShoulderY, baseShoulderZ)

            shoulderElbowX = (self.shoulderL.location[0], self.elbowL.location[0])
            shoulderElbowY = (self.shoulderL.location[1], self.elbowL.location[1])
            shoulderElbowZ = (self.shoulderL.location[2], self.elbowL.location[2])
            shoulderElbow = plt3d.art3d.Line3D(shoulderElbowX, shoulderElbowY, shoulderElbowZ)

            elbowWristX = (self.elbowL.location[0], self.wristL.location[0])
            elbowWristY = (self.elbowL.location[1], self.wristL.location[1])
            elbowWristZ = (self.elbowL.location[2], self.wristL.location[2])
            elbowWrist = plt3d.art3d.Line3D(elbowWristX, elbowWristY, elbowWristZ)

            wristHandX = (self.wristL.location[0], self.handL.location[0])
            wristHandY = (self.wristL.location[1], self.handL.location[1])
            wristHandZ = (self.wristL.location[2], self.handL.location[2])
            wristHand = plt3d.art3d.Line3D(wristHandX, wristHandY, wristHandZ)

        elif side == "R":
            baseShoulderX = (self.baseR.location[0], self.shoulderR.location[0])
            baseShoulderY = (self.baseR.location[1], self.shoulderR.location[1])
            baseShoulderZ = (self.baseR.location[2], self.shoulderR.location[2])
            baseShoulder = plt3d.art3d.Line3D(baseShoulderX, baseShoulderY, baseShoulderZ)

            shoulderElbowX = (self.shoulderR.location[0], self.elbowR.location[0])
            shoulderElbowY = (self.shoulderR.location[1], self.elbowR.location[1])
            shoulderElbowZ = (self.shoulderR.location[2], self.elbowR.location[2])
            shoulderElbow = plt3d.art3d.Line3D(shoulderElbowX, shoulderElbowY, shoulderElbowZ)

            elbowWristX = (self.elbowR.location[0], self.wristR.location[0])
            elbowWristY = (self.elbowR.location[1], self.wristR.location[1])
            elbowWristZ = (self.elbowR.location[2], self.wristR.location[2])
            elbowWrist = plt3d.art3d.Line3D(elbowWristX, elbowWristY, elbowWristZ)

            wristHandX = (self.wristR.location[0], self.handR.location[0])
            wristHandY = (self.wristR.location[1], self.handR.location[1])
            wristHandZ = (self.wristR.location[2], self.handR.location[2])
            wristHand = plt3d.art3d.Line3D(wristHandX, wristHandY, wristHandZ)

        toReturn = [baseShoulder, shoulderElbow, elbowWrist, wristHand]
        return toReturn


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
        toReturn = Rotation.from_quat(self.rotation)
        toReturn = toReturn.as_matrix()
        return toReturn

    def toHomoMatrix(self):
        rotMtx = self.toRotationMatrix()
        toReturn = np.eye(4)
        toReturn[0:3, 0:3] = rotMtx
        toReturn[0:2, 3] = self.translation
        return toReturn


class LocationAndOrientation(QuatAndTransform):

    def __init__(self, translation, rotation):
        assert len(translation) == 3, "translation must contain 3 elements"
        assert len(rotation) == 4, "rotation quaternion must contain 4 elements"
        super().__init__(translation, rotation)
