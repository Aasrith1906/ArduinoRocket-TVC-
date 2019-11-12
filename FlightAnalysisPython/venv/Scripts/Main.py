#import matplotlib.pyplot as plt
#import numpy as np
#import pandas as pd
import sys
import os

def OpenFile(file_name):

    file_dir = os.path.join(os.getcwd() + file_name)

    try:

        file = open(file_dir , 'r')

        return file

    except Exception as e:

        print(str(e))

        return -1

        sys.exit(0)


def GetData(file):

    data = ""
    #data = pd.dataframe()

    return data

def PlotData(data):

    pass

if __name__ == '__main__':

    if len(sys.argv) < 2:

        print("usage: python Main.py <file_name>")

        sys.exit(0)

    file_name = str(sys.argv[1])

    try:

        file = open(file_name)

        data = GetData(file)

        PlotData(data)

    except Exception as e:

        print(str(e))

        sys.exit(0)
