import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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
    df = pd.read_csv(filePath, index_col=0)
    df.rename(columns={"0": "Shoulder Pitch", "1": "Shoulder Roll", "2": "Elbow Yaw", "3": "Elbow Roll"},
              errors="raise", inplace=True)


