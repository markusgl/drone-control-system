import json
import os
from pprint import pprint
import optparse
import sys
from PIL import Image

parser = optparse.OptionParser()

parser.add_option('-n', '--num', action="store", dest="num_classes", help="Number of classes", default=5)
parser.add_option('-p', '--path', action="store", dest="path", help="Path to img and json", default="./data/train_original")

options, args = parser.parse_args()

num_classes = int(options.num_classes)

# for line in open(os.path.join(os.path.dirname(__file__), 'data/train_original/data.json'), 'r'):
for line in open(options.path + '/data.json', 'r'):
    json_line = json.loads(line)
    click_positions = json_line["click-Positions"]
    resolution = json_line["resolution"]
    crop_size = resolution[1] / num_classes
		
    if len(click_positions) == 2:
        # oberen Click finden
        if click_positions[0][1] < click_positions[1][1]:
            absolute_rope_position = click_positions[0][0]
        else:
            absolute_rope_position = click_positions[1][0]

        relative_rope_position = absolute_rope_position / resolution[1]
        
        rope_class = int(relative_rope_position * num_classes)
        if rope_class == num_classes:
            rope_class -= 1
        
        img = Image.open(options.path + '/' + json_line["filename"])
        
        for i in range(0, num_classes -1):
            area = (i * crop_size, 0, (i + 1) * crop_size, crop_size)
            cropped_img = img.crop(area)
            
            if i == rope_class:
                new_path = '/rope/'
            else:
                new_path = '/norope/'
            
            cropped_img.save(options.path + new_path + str(i) + '_' + json_line["filename"])
        
        # sys.stdout.write(str(rope_class))