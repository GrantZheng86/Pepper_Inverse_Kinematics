from DataAnalysis import plotJointAngles, animateArm

if __name__ == "__main__":
    fileName = "optimization_L.csv"
    plotJointAngles(fileName)
    animateArm(fileName, "setpoints_L.csv")
    