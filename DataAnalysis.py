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
    # ax = fig.add_subplot(111, projection='3d')
    ax = fig.gca(projection='3d')

    # xL, yL, zL = pepperModel.get3DLine('L')
    # xR, yR, zR = pepperModel.get3DLine('R')
    # print(xL, yL, zL)
    # print(xR, yR, zR)
    # ax.plot(xL,yL,zL)
    # ax.plot(xR, yR, zR)
    # plt.show()

    for i in range(l):
        ax.clear()
        ax.set_xlim3d(-.25, 0.5)
        ax.set_ylim3d(-.25, 0.5)
        ax.set_zlim3d(-.25, 0.5)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        shoulderPitch = jointDF['Shoulder Pitch'][i]
        shoulderRoll = jointDF['Shoulder Roll'][i]
        elbowYaw = jointDF['Elbow Yaw'][i]
        elbowRoll = jointDF['Elbow Roll'][i]

        pepperModel.shoulderPitchL.updateTransforms(0., shoulderPitch, 0.)
        pepperModel.shoulderRollL.updateTransforms(0, 0, shoulderRoll)
        pepperModel.elbowYawL.updateTransforms(elbowYaw, 0, 0)
        pepperModel.elbowRollL.updateTransforms(0, 0, elbowRoll)
        pepperModel.updateJointLocation()

        xS, yS, zS = pepperModel.get3DLine("L")

        ax.plot(xS, yS, zS)
        ax.plot([setPointsDF['X'][i]], [setPointsDF['Y'][i]], [setPointsDF['Z'][i]],
                markerfacecolor='r', markeredgecolor='k', marker='o', markersize=5, alpha=0.6)

        ax.plot([0], [0], [0],
                markerfacecolor='b', markeredgecolor='k', marker='o', markersize=6, alpha=0.6)

        plt.pause(0.001)
