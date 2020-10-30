import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import HelperClasses
import mpl_toolkits.mplot3d as plt3d

dataPath = "C:\\Users\\Zheng\\OneDrive - Colorado School of Mines\\Pepper IK\\"
dataFolder = "Run 2 Data\\"


def plotJointAngles(fileName, showPlot=True):
    filePath = dataPath + dataFolder + fileName
    df = pd.read_csv(filePath, index_col=0)
    df.rename(columns={"0": "Shoulder Pitch", "1": "Shoulder Roll", "2": "Elbow Yaw", "3": "Elbow Roll"},
              errors="raise", inplace=True)
    fig, subplots = plt.subplots(2, 2)
    subplots[0][0].plot(df['Shoulder Pitch'])
    subplots[0][1].plot(df['Shoulder Roll'])
    subplots[1][0].plot(df['Elbow Yaw'])
    subplots[1][1].plot(df['Elbow Roll'])

    colNames = df.columns
    subplots[0][0].set_title(colNames[0])
    subplots[0][1].set_title(colNames[1])
    subplots[1][0].set_title(colNames[2])
    subplots[1][1].set_title(colNames[3])

    fig.suptitle(fileName)

    if showPlot:
        plt.show()


def animateArm(jointConfigurations, setPoints):
    # The transform below are all in ** METERS **

    filePath = dataPath + dataFolder + jointConfigurations
    setPointsPath = dataPath + dataFolder + setPoints
    jointDF = pd.read_csv(filePath, index_col=0)
    jointDF.rename(columns={"0": "Shoulder Pitch", "1": "Shoulder Roll", "2": "Elbow Yaw", "3": "Elbow Roll"},
                   errors="raise", inplace=True)
    setPointsDF = pd.read_csv(setPointsPath, index_col=0)
    setPointsDF.rename(columns={"0": "X", "1": "Y", "2": "Z"}, errors="raise",
                       inplace=True)
    pepperModel = HelperClasses.Pepper()
    l = len(setPointsDF.index)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    armSegments3DL = pepperModel.get3DLine('L')
    armSegments3DR = pepperModel.get3DLine('R')
    for each_line in armSegments3DL:
        ax.add_line(each_line)

    for each_line in armSegments3DR:
        ax.add_line(each_line)

    for i in range(l):
        shoulderPitch = jointDF['Shoulder Pitch'][i]
        shoulderRoll = jointDF['Shoulder Roll'][i]
        elbowYaw = jointDF['Elbow Yaw'][i]
        elbowRoll = jointDF['Elbow Roll'][i]

        pepperModel.shoulderL.updateTransforms(0., shoulderPitch, shoulderRoll)
        pepperModel.elbowL.updateTransforms(elbowYaw, 0., elbowRoll)

    print(pepperModel)
