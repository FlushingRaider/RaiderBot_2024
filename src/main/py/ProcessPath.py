import json, os

#open our file

working_path = "C:/Users/sdbig/Documents/Source/C_Source/RaiderBot_2023/RaiderBot_2023/src/main/deploy/paths/output/"
working_name = "BlueP1"

output_path = "C:/Users/sdbig/Documents/Source/C_Source/RaiderBot_2023/RaiderBot_2023/src/main/include/MotionProfiles/"
output_name = working_name + ".hpp"

json_file = working_path + working_name + ".wpilib.json"

rotation_start = 0
rotation_end = 90

with open(json_file) as json_data:
    data = json.load(json_data) 

#time
time_data = ""
time_size = 0
time_final = ""

for elm in data:
    time_data += str(elm["time"]) + ","
    time_final = str(elm["time"])
    time_size += 1

time_head = "const double " + working_name + "_T" + "[" + str(time_size) + "] = {"
time_foot = "};"

time_output = time_head + time_data + time_foot

#rot
rot_data = ""
rot_time_data = ""
rot_size = 2

#for elm in data:
#    rot_data += str(elm["pose"]["rotation"]["radians"]) + ","
#    rot_size += 1

rot_data = str(rotation_start) + "," + str(rotation_end)
rot_time_data = str(0) + "," + time_final

rot_head = "const double " + working_name + "_ROT" + "[" + str(rot_size) + "] = {"
rot_foot = "};"

rot_output = rot_head + rot_data + rot_foot

rot_time_head = "const double " + working_name + "_ROT_T" + "[" + str(rot_size) + "] = {"
rot_foot = "};"
rot_time_output = rot_time_head + rot_time_data + rot_foot

#x
x_data = ""
x_size = 0

for elm in data:
    x_data += str(elm["pose"]["translation"]["x"]) + ","
    x_size += 1

x_head = "const double " + working_name + "_X" + "[" + str(x_size) + "] = {"
x_foot = "};"

x_output = x_head + x_data + x_foot

#y
y_data = ""
y_size = 0

for elm in data:
    y_data += str(elm["pose"]["translation"]["y"]) + ","
    y_size += 1

y_head = "const double " + working_name + "_Y" + "[" + str(y_size) + "] = {"
y_foot = "};"

y_output = y_head + y_data + y_foot

with open(output_path + output_name, "a") as f:
    print(time_output, file=f)
    print(rot_time_output, file=f)
    print(rot_output, file=f)
    print(x_output, file=f)
    print(y_output, file=f)