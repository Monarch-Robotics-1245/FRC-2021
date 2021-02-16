# for converting old csvs to include the backwards parameter
import sys

pathName = sys.argv[1]

with open("src/main/deploy/"+pathName+".csv") as f:
  data = f.read().split("\n")

file = open("src/main/deploy/"+pathName+".csv", "w")

for d in data:
    file.write(d + ",0" + "\n")
