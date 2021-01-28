from networktables import NetworkTablesInstance, NetworkTables
import time

NetworkTables.initialize(server="roborio-1245.frc.local")
nt = NetworkTables.getTable("Position")

f = open("position.csv", "a")


while True:
    x = nt.getNumber("x")
    y = nt.getNumber("y")
    rot = nt.getNumber("rotation")
    print("x: "+str(x))
    print("y: "+str(y))
    print("rot: "+str(rot))
    f.write(str(x)+","+str(y)+","+str(rot))
    time.sleep(0.100)

