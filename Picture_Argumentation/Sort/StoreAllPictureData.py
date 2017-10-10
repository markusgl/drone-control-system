import os
from JSONReader import JSONReader
import sys
import shutil
import json


def getDataFromJsonFile():
    jsonFileReader=JSONReader()
    jsonData=jsonFileReader.read()
    path= jsonFileReader.getPath()
    returnData={}
    returnData['jsonData']=jsonData
    returnData['jsonPath']=path
    return returnData

def addFullPath(data):
    oldJsonData=data['jsonData']
    path= data['jsonPath']
    newJsonData = []
    for tupel in oldJsonData:
        tupel['filename'] = path +'/'+ tupel['filename']
        newJsonData.append(tupel)
    return newJsonData

def calcRopePos(inputData):
    newJsonData = []
    for tuple in inputData:
        pictureName = tuple['filename']
        pictureSize = tuple['resolution']
        clickpositions = tuple['click-Positions']

        if not clickpositions:
            print("Picture %s wird in Kategorie %s sortiert." % (pictureName, 'noRope'))
        elif len(clickpositions) == 2:
            firstClick = clickpositions[0]
            secondClick = clickpositions[1]

            if firstClick[1] < secondClick[1]:
                uppoerClick = firstClick
                lowerClick = secondClick
            else:
                uppoerClick = secondClick
                lowerClick = firstClick

            pictureWith= pictureSize[1]
            middle= pictureWith/2

            pos=(middle -uppoerClick[0])/middle
            tuple['ropePos']=round(pos,4)
            newJsonData.append(tuple)
    return newJsonData

if __name__ == '__main__':
    test =  getDataFromJsonFile()
    d= addFullPath(test)
    e = calcRopePos(d)

    with open('Newdata.json', 'a') as outfile:
        outfile.writelines(json.dumps(item) + '\n' for item in e)