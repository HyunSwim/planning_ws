import rospy
from new_gigacha.msg import Path
import csv
from numpy import rad2deg

def read_local_path(where, name):
        print(f"Making Local Path from {where}/{name}.csv")
        local_path = Path()
        with open("map/" + where + "/" + name + ".csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                local_path.x.append(float(line[0]))
                local_path.y.append(float(line[1]))
                deg_yaw=(rad2deg(float(line[2]))+360) % 360
                local_path.heading.append(deg_yaw)
                local_path.k.append(float(line[3]))
                # local_path.env.append(line[5])
                # local_path.mission.append(line[6])
        return local_path