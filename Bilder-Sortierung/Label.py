import numpy as np
import cv2
import os
import json
import ctypes


#File Dialog
from tkinter.filedialog import askdirectory
filename = askdirectory();

user32 = ctypes.windll.user32
screen_res = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

#Set new Window Size
def setSize():
    new_width = screen_res[0] *0.5
    new_height = screen_res[1] *0.5
    print(new_height, new_height)
    window_width = int(new_width)
    window_height =int(new_height)

    cv2.resizeWindow('image', window_width, window_height)
    return

#Fenstergröße zum Anzeigen der Bilder auf halbe Bildschirmgröße
setSize()


#Alle jpg Files in dem gewählten Verzeichnis suchen
pictures=[]
for file in os.listdir(filename):
    if file.split(".")[-1] =="jpg":
        print (file)
        pictures.append(filename+"/"+file)


#the [x, y] for each right-click event will be stored here
right_clicks = []
counter=0

#this function will be called whenever the mouse is right-clicked
def mouse_callback(event, x, y, flags, params):

    global counter
    #right-click event value is 2
    if event == 1 and counter <2:

        global right_clicks
        counter+=1
        #store the coordinates of the right-click event
        right_clicks.append([x, y])




#create Window
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
#set mouse callback function for window
cv2.setMouseCallback('image', mouse_callback)


#Dictonary für Dateiname,Auflösung und Positionen der Click
imgData={}
jsonData=[]
imgCounter=0

#Jedes Bild in dem Ordner wir geöffnet, Clicks werden registriert.
for pic in pictures:

            #Click-Counter wieder nullen
            counter = 0

            #Bild öffnen und anzeigen
            img = cv2.imread(pic, 1)
            cv2.imshow('image', img)

            # wartet auf irgendeine Taste oder ESC dann wird
            # das Programm beendet
            esc = cv2.waitKey(0)
            if esc == 27:
                break
            #Äuflösung und Pfadname in Dict speichern
            imgData={
                'filename': os.path.basename(pic),
                # Achtung, img.shape liefert Höhe x Breite x Channels
                'resolution': img.shape,
                'click-Positions': right_clicks
            }
            print(imgData)
            jsonData.append(imgData)
            imgCounter+=1
            right_clicks=[]


with open('data.json', 'a') as outfile:
    outfile.writelines(json.dumps(item)+ '\n' for item in jsonData)

cv2.destroyAllWindows()