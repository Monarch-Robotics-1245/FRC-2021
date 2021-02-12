# usage: py convert-pathweaver.py [Name of path]
# ex: py convert-pathweaver.py Slalom

import json
import sys

pathName = sys.argv[1]

with open('src/main/deploy/paths/'+pathName+'.wpilib.json') as f:
  data = json.load(f)
initialX = data[0]['pose']['translation']['x']
initialY = data[0]['pose']['translation']['y']

file = open("src/main/deploy/"+pathName+".csv", "w")
csvData = []

for d in data:
    x = d['pose']['translation']['x'] - initialX
    y = d['pose']['translation']['y'] - initialY
    csvData.append(str(x) + "," + str(y) + ",1.0,0,0")

file.write("\n".join(csvData))

print("Length: "+str(len(data)))