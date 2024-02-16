import json, os
import pdb

#open our file

#pdb.set_trace()
working_path = "C:/Users/gnich/Documents/RaiderBot_2024/src/main/pathing/Output_Directory/"
working_name = "Precision_Test"

output_path = "C:/Users/gnich/Documents/RaiderBot_2024/src/main/include/MotionProfiles/"
output_name = working_name + ".hpp"

json_file = working_path + working_name + ".wpilib.json"

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

time_head = "const double KnADAS_t_" + working_name + "[" + str(time_size) + "] = {"
time_foot = "};"

time_output = time_head + time_data + time_foot

#rot
rot_data = ""
time_remain_data = ""
rot_size = 0

for elm in data:
   data_deg = (elm["holonomicRotation"])
   rot_data += str(data_deg) + ","
   rot_size += 1

for elm in data:
    time_remain_data += str(float(time_final) - float(elm["time"])) + ","
    


rot_head = "const double KaADAS_Deg_" + working_name + "[" + str(rot_size) + "] = {"
rot_foot = "};"

rot_output = rot_head + rot_data + rot_foot

time_remain_head = "const double KaADAS_t_" + working_name + "Remaining" + "[" + str(rot_size) + "] = {"
rot_time_output = time_remain_head + time_remain_data + rot_foot

#x
x_data = ""
x_size = 0

for elm in data:
    data_in = (elm["pose"]["translation"]["y"] * (39.37008))
    x_data += str(data_in) + ","
    x_size += 1

x_head = "const double KaADAS_l_" + working_name + "_X" + "[" + str(x_size) + "] = {"
x_foot = "};"

x_output = x_head + x_data + x_foot

#y
y_data = ""
y_size = 0

for elm in data:
    data_in = (elm["pose"]["translation"]["x"] * (39.37008))
    y_data += str(data_in) + ","
    y_size += 1

y_head = "const double KaADAS_l_" + working_name + "_Y" + "[" + str(y_size) + "] = {"
y_foot = "};"

y_output = y_head + y_data + y_foot

with open(output_path + output_name, "a") as f:
    print(time_output, file=f)
    print(rot_time_output, file=f)
    print(rot_output, file=f)
    print(x_output, file=f)
    print(y_output, file=f)
