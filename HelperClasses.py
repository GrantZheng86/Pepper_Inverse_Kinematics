from scipy.spatial.transform import Rotation
import numpy as np

class Pepper:
    def __init__(self):
        baseToShoulder = {
            'L': Transform([-0.057, 0.14974, 0.08682],[0., 0., 0., 1.]),
            'R': Transform([-0.057, -0.14974, 0.08682],[0., 0., 0., 1.])
        }

        shoulderToElbow = {
            'L': Transform([0.1812, 0.015, 0.00013],[0., 0., 0., 1.]),
            'R': Transform([0.1812, -0.015, 0.00013],[0., 0., 0., 1.])
        }

        elbowToWrist = {
            'L': Transform([0.15, 23.6e-3, 22.84e-3],[ 0, -0.0784593, 0, 0.9969173 ]),
            'R': Transform([0.15, 23.6e-3, 22.84e-3],[ 0, 0.0784593, 0, 0.9969173 ])
        }

        wristToHand = {
            'L': Transform([0.0695, 0.0, -0.03030],[0., 0., 0., 1.]),
            'R': Transform([0.0695, 0.0, -0.03030],[0., 0., 0., 1.])
        }
        self.baseL = JointWithTransforms(None, baseToShoulder['L'])
        self.baseR = JointWithTransforms(None, baseToShoulder['R'])

        self.shoulderL = JointWithTransforms(None, shoulderToElbow['L'])
        self.shoulderR = JointWithTransforms(None, shoulderToElbow['R'])

        self.baseL.toFrame = self.shoulderL
        self.baseR.toFrame = self.shoulderR

        self.elbowL = JointWithTransforms(None, elbowToWrist['L'])
        self.elbowR = JointWithTransforms(None, elbowToWrist['R'])

        self.shoulderL.toFrame = self.elbowL
        self.shoulderR.toFrame = self.elbowR

        self.wristL = JointWithTransforms(None, wristToHand['L'])
        self.wristR = JointWithTransforms(None, wristToHand['R'])

        self.elbowL.toFrame = self.wristL
        self.elbowR.toFrame = self.wristR

        self.handL = JointWithTransforms(None, None)
        self.handR = JointWithTransforms(None, None)

        self.wristL.toFrame = self.handL
        self.wristR.toFrame = self.handR


class JointWithTransforms:

    def __init__(self, toFrame, transform):
        self.toFrame = toFrame
        self.transform = transform


class Transform:

    def __init__(self, translation, rotation):
        assert len(translation) == 3, "translation must contain 3 elements"
        assert len(rotation) == 4, "rotation quaternion must contain 4 elements"

        self.translation = translation
        self.rotation = rotation

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
