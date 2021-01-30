# usage: py convert-pathweaver.py [Name of path]
# ex: py convert-pathweaver.py Slalom

import json
import sys

pathName = sys.argv[1]

with open('src/main/deploy/paths/'+pathName+'.wpilib.json') as f:
  data = json.load(f)
print("Length: "+str(len(data)))
initialX = data[0]['pose']['translation']['x']
initialY = data[0]['pose']['translation']['y']

file = open("src/main/deploy/"+pathName+".csv", "w")

for d in data:
    x = d['pose']['translation']['x'] - initialX
    y = d['pose']['translation']['y'] - initialY
    rot = d['pose']['rotation']['radians']
    file.write(str(x) + "," + str(y) + "," + str(rot) + "\n")