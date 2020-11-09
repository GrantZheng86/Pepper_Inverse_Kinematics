from DataAnalysis import plotJointAngles, animateArm
from HelperClasses import QuatAndTransform, Pepper
def testIK():
    setPoint = [-0.103819538,	0.155992683,	-0.331068363]
    setOrient = [0, 0, 0, 1]
    desiredLocPose = QuatAndTransform(setPoint,setOrient)
    pepper = Pepper()
    pepper.moveTo(desiredLocPose, 'L')



if __name__ == "__main__":
    # fileName = "optimization_L.csv"
    # plotJointAngles(fileName)
    # animateArm(fileName, "setpoints_L.csv")
    testIK()



    