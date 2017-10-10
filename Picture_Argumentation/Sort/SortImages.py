from JSONReader import JSONReader
from PictureSorter import PictureSorter


reader= JSONReader()
jsonData=reader.read()

sorter=PictureSorter(jsonData, reader.getPath())
sorter.sort()



