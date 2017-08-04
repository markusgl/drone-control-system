import json
import os
import sys

PY3K = sys.version_info >= (3,)
if PY3K:
    from tkinter import filedialog as fd
else:
    from Tkinter import tkFileDialog as fd


class JSONReader:

    def __init__(self):
        self.jsonPath=fd.askopenfilename()
        self.jsonData=[]

    def getPath(self):
        return os.path.dirname(self.jsonPath)

    def __openAndReadJsonFile(self):
        for line in open(self.jsonPath, 'r'):
            self.jsonData.append(json.loads(line))

    def read(self):
        self.getPath()
        self.__openAndReadJsonFile()
        return self.jsonData