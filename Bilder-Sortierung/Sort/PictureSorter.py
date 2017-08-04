import os
from JSONReader import JSONReader
import sys


class PictureSorter:

    def __init__(self, jsonData, path):
        self.jsonData=jsonData
        self.categoriesNames=[]
        self.categories={}
        self.path=path

    # diese Methode fordert den Benutzer auf Kategoriebezeichnungen einzugeben
    # wird 'esc' als String eingegeben wird die Schleife beendet
    # tpdo: sollte irgendwie auf ESC - Taste überprüf werden.
    def __askCategorieNames(self):
        counter = 1
        abort = False
        while not abort :
            inputString = input("Geben Sie die Bezeichnung für die %d Kategorie ein: " % (counter))

            if inputString !='esc':
                self.categoriesNames.append(inputString)
                counter = counter + 1
            else:
                abort=True
                print("=================================================")
                print()

        self.__createCategories()



    # Hier werdne die Kategoriegrenzen festgelegt
    # im Moment wird nur eine Lineareverteilung unterstüzt
    def __createCategories(self):

        #inputs= input("Soll eine lineare Verteilung der Categoriene durchgeführt werden [j / n ]")
        inputs='j'
        if inputs=='j':

            categorieCount= len(self.categoriesNames)
            length= 1/categorieCount

            start=0
            end=0
            for categorieName in self.categoriesNames:
                categorie={}
                start= end
                end=start+length
                categorie['begin']=start
                categorie['end']=end
                self.categories[categorieName]= categorie
                print("Kategorie %s beginnt bei %f und endet bei %f" % (categorieName, start*100, end*100))
        print("=================================================")
        print()


    # Erstellt anhand der eingegeben Kategoriebezeichnungen Ordner im selbem
    # Pfad in dem das JSON File liegt
    def __makeDirectorys(self, path):

        self.__askCategorieNames()

        for categorie in self.categoriesNames:
            directory= os.path.join(path,categorie)

            if not os.path.exists(directory):
                os.makedirs(directory)



    def sort(self):

        self.__makeDirectorys(self.path)
        for data in self.jsonData:
            pictureName = data['filename']
            pictureSize = data['resolution']
            clickpositions = data['click-Positions']

            if not clickpositions:
                print("no Rope")
            else:
                firstClick = clickpositions[0]
                secondClick = clickpositions[1]

                if firstClick[1] < secondClick[1]:
                    oben = firstClick
                    unten = secondClick
                else:
                    oben = secondClick
                    unten = firstClick

                relativRopePos=oben[0]/pictureSize[1]
                print("Picture %s wird in Kategorie %s sortiert. Seilposition gefunden bei ca. %f " % (pictureName,self.__categrorize(relativRopePos), relativRopePos*100))


    # Diese Methode prüft in wechle Kategorie das jeweilige Bild eingeordnet werden soll
    def __categrorize(self, position):
        #https://stackoverflow.com/questions/3294889/iterating-over-dictionaries-using-for-loops
        for key, value in self.categories.items():

            if position > value['begin'] and position < value['end']:
                return key




