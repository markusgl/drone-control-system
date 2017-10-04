import json
import os
from pprint import pprint
import optparse

parser = optparse.OptionParser()

parser.add_option('-c', '--csv', action="store", dest="csv_name", help="Path to CSV file", default="data.csv")
parser.add_option('-j', '--json', action="store", dest="json_name", help="Path to JSON file", default="data.json")

options, args = parser.parse_args()

csv = open(options.csv_name, "a")

if os.path.getsize(options.csv_name) == 0:
  csv.write("filename,width,height,class,xmin,ymin,xmax,ymax\n")

images = []

click_positions = []

for line in open(options.json_name, 'r'):
	images.append(json.loads(line))

for image in images:
		click_positions = image["click-Positions"]
		if len(click_positions) == 2:
			x = [click_positions[0][0], click_positions[1][0]]
			y = [click_positions[0][1], click_positions[1][1]]
			
			csv.write(image["filename"] +  ',' + str(image["resolution"][1]) + ',' + str(image["resolution"][0]) + ',Seil,' + str(min(x)) + ',' + str(min(y)) + ',' + str(max(x)) + ',' + str(max(y)) + '\n')
			
csv.close()