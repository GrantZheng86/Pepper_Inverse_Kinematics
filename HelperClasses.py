from scipy.spatial.transform import Rotation
import numpy as np
from abc import ABC, abstractclassmethod
import scipy

import mpl_toolkits.mplot3d as plt3d


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
            'L': Transform([0.15, -23.6e-3, 22.84e-3], [0, -0.0784593, 0, 0.9969173]),
            'R': Transform([0.15, 23.6e-3, 22.84e-3], [0, 0.0784593, 0, 0.9969173])
        }

        """
        NOTE: The wrist to hand commented out below uses the points from softbank documentation
        , which is in the world frame. Since there is an elbow offset angle for pepper in the 
        zero position, the transformation has been updated to the new one below
        """
        # wristYaw2hand = {
        #     'L': Transform([0.0695, 0.0, -0.03030], [0., 0., 0., 1.]),
        #     'R': Transform([0.0695, 0.0, -0.03030], [0., 0., 0., 1.])
        # }

        wristYaw2hand = {
            'L': Transform([0.06390450580, 0.0, -0.04079881526], [0., 0., 0., 1.]),
            'R': Transform([0.07338414596, 0.0, -0.01905508616], [0., 0., 0., 1.])
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
                                                 location=None)
        self.shoulderRollR = JointWithTransforms('shoulderRollR', None,
                                                 shoulderRoll2elbowYaw['R'],
                                                 location=None)

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
                                              location=None)
        self.elbowRollR = JointWithTransforms('elbowRollR', None,
                                              elbowRoll2wristYaw['R'],
                                              location=None)

        self.elbowYawL.toJoint = self.elbowRollL
        self.elbowYawR.toJoint = self.elbowRollR

        self.wristYawL = JointWithTransforms('wristYawL', None,
                                             wristYaw2hand['L'],
                                             location=None)
        self.wristYawR = JointWithTransforms('wristYawR', None,
                                             wristYaw2hand['R'],
                                             location=None)

        self.elbowRollL.toJoint = self.wristYawL
        self.elbowRollR.toJoint = self.wristYawR

        self.handL = JointWithTransforms('handL', None, None, location=None)
        self.handR = JointWithTransforms('handR', None, None, location=None)

        self.wristYawL.toJoint = self.handL
        self.wristYawR.toJoint = self.handR

        self.updateJointLocation()
        # self._initJointLocations()

    def _initJointLocations(self):

        currJoint = self.baseL
        while currJoint.toJoint is not None:
            currJoint.toJoint.location = np.add(currJoint.location, currJoint.transform.translation)
            currJoint = currJoint.toJoint

        currJoint = self.baseR
        while currJoint.toJoint is not None:
            currJoint.toJoint.location = np.add(currJoint.location, currJoint.transform.translation)
            currJoint = currJoint.toJoint

    def updateJointLocation(self):

        start = np.eye(4)
        currentJoint = self.baseL

        while currentJoint.toJoint is not None:
            transform2Next = currentJoint.transform.toHomoMatrix()
            start = np.matmul(start, transform2Next)
            currentJoint.toJoint.location = start[0:3, 3]
            currentJoint = currentJoint.toJoint

        currentJoint = self.baseR
        start = np.eye(4)
        while currentJoint.toJoint is not None:
            start = np.matmul(start, currentJoint.transform.toHomoMatrix())
            currentJoint.toJoint.location = start[0:3, 3]
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

            currJoint = self.baseL
            while currJoint.toJoint is not None:
                currX = [currJoint.location[0], currJoint.toJoint.location[0]]
                currY = [currJoint.location[1], currJoint.toJoint.location[1]]
                currZ = [currJoint.location[2], currJoint.toJoint.location[2]]
                currJoint = currJoint.toJoint
                for i in range(len(currX)):
                    xToReturn.append(currX[i])
                    yToReturn.append(currY[i])
                    zToReturn.append(currZ[i])


        elif side == "R":

            currJoint = self.baseR
            while currJoint.toJoint is not None:
                currX = [currJoint.location[0], currJoint.toJoint.location[0]]
                currY = [currJoint.location[1], currJoint.toJoint.location[1]]
                currZ = [currJoint.location[2], currJoint.toJoint.location[2]]
                currJoint = currJoint.toJoint
                for i in range(len(currX)):
                    xToReturn.append(currX[i])
                    yToReturn.append(currY[i])
                    zToReturn.append(currZ[i])

        return xToReturn, yToReturn, zToReturn

    def fkAndJacob(self, side):

        toReturn = np.zeros((6,5))
        jointAxises = np.array([[0, 1, 0],
                                [0, 0, 1],
                                [1, 0, 0],
                                [0, 0, 1],
                                [1, 0, 0],
                                [0, 0, 0]])

        jointA = np.eye(4)
        if side == 'L':
            currJoint = self.baseL
        elif side =='R':
            currJoint = self.baseR
        index = 0
        jointDir = np.transpose([0, 0, 0])
        upperHalfJacob = []
        lowerHalfJacob = []
        handPose = None
        while currJoint.toJoint is not None:
            handLoc = self.handL.location
            currJointLoc = currJoint.location
            difference = np.subtract(handLoc, currJointLoc)
            jointA = np.matmul(jointA, currJoint.transform.toHomoMatrix())
            jointDir = np.matmul(jointA[0:3, 0:3], np.transpose(jointAxises[index]))
            upperHalfJacob.append(np.cross(jointDir, difference))
            lowerHalfJacob.append(jointDir)
            currJoint = currJoint.toJoint

            if currJoint.toJoint is None:
                handPose = jointA
            index += 1

        toReturn = np.array(toReturn)
        upperHalfJacob = np.array(upperHalfJacob)
        lowerHalfJacob = np.array(lowerHalfJacob)
        toReturn[0:3, :] = np.transpose(upperHalfJacob[:-1, :])
        toReturn[3:6, :] = np.transpose(lowerHalfJacob[:-1, :])

        return toReturn, handPose


    def calculateError(self, desiredPose, side):

        jacob, handPose = self.fkAndJacob(side)
        r = Rotation.from_matrix(handPose[0:3, 0:3])
        r = r.as_rotvec()
        t = handPose[0:3, 3]

        desiredR = Rotation.from_quat(desiredPose.quaternion)
        desiredR = desiredR.as_rotvec()
        desiredT = desiredPose.translation

        rDiff = np.subtract(r, desiredR)
        tDiff = np.subtract(t, desiredT)

        # tWeight = 1.0
        # rWeight = 0.0
        # rErr = rWeight * np.sum(np.power(rDiff, 2))
        # tErr = tWeight * np.sum(np.power(tDiff, 2))

        return np.stack((tDiff,rDiff))

    def changeJointAngle(self, jointAngles, side):

        index = 0

        currJoint = None
        if side == "L":
            currJoint = self.baseL.toJoint
        elif side == 'R':
            currJoint = self.baseR.toJoint

        jointAxis = np.array([[0, 1, 0],
                              [0, 0, 1],
                              [1, 0, 0],
                              [0, 0, 1],
                              [1, 0, 0]])

        while currJoint.toJoint is not None:
            currJointAngle = np.multiply(jointAxis[index], jointAngles[index])
            currJoint.updateTransforms(currJointAngle[0], currJointAngle[1], currJointAngle[2])
            currJoint = currJoint.toJoint
            index += 1

        self.updateJointLocation()



    def moveTo(self, desiredPose, side):

        prevErrNorm = np.inf
        currErr = self.calculateError(desiredPose, side)
        currErrT = currErr[0]
        currErrNorm = np.linalg.norm(currErrT)
        stepSize = 0.01

        while currErrNorm < prevErrNorm:
            jacob, pose = self.fkAndJacob(side)
            jacob = jacob[0:3, :] # take the upper jacobian matrix for inverse kinematics
            currErr = self.calculateError(desiredPose, side)
            tErr = currErr[0]   # take only the position err
            currErrNorm = np.linalg.norm(tErr)

            # NOTE: The jacobian used here is only the upper part
            # TODO: Test the IK use full jacobian
            jInv = np.linalg.pinv(jacob)
            delTheta = np.matmul(jInv, np.transpose(tErr))
            delTheta = np.multiply(delTheta, stepSize)
            self.changeJointAngle(delTheta, side)

            prevErrNorm = currErrNorm
            currErr = self.calculateError(desiredPose, side)
            tErr = currErr[0]
            currErrNorm = np.linalg.norm(tErr)










class QuatAndTransform(ABC):

    def __init__(self, translation, quaternion):
        self.translation = translation
        self.quaternion = quaternion
        super().__init__()


class JointWithTransforms:


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
