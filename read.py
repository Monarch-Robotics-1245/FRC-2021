from networktables import NetworkTablesInstance, NetworkTables
import time

NetworkTables.initialize(server="roboRIO-1245-frc.local")
nt = NetworkTables.getTable("Position")

f = open("position.csv", "w")
f.write("x,y,rot\n")


while True:
    x = nt.getNumber("x", 0.0)
    y = nt.getNumber("y", 0.0)
    rot = nt.getNumber("rotation", 0.0)
    # print("x: "+str(x))
    # print("y: "+str(y))
    # print("rot: "+str(rot))
    f.write(str(x)+","+str(y)+","+str(rot)+"\n")
    time.sleep(0.20)

