import json, os
import pdb

#open our file

#pdb.set_trace()
working_path = "C:/Users/scott/Documents/Source/C_Source/RaiderBot_2024/RaiderBot_2024/src/main/include/OptPaths/"
working_name = "Opt1Path5"

output_path = "C:/Users/scott/Documents/Source/C_Source/RaiderBot_2024/RaiderBot_2024/src/main/include/OptPaths/"
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

time_head = "const std::vector<double> KnADAS_t_" + working_name + " = {"
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
    


rot_head = "const std::vector<double> KaADAS_Deg_" + working_name + " = {"
rot_foot = "};"

rot_output = rot_head + rot_data + rot_foot

time_remain_head = "const std::vector<double> KaADAS_t_" + working_name + "Remaining" + " = {"
rot_time_output = time_remain_head + time_remain_data + rot_foot

#x
x_data = ""
x_size = 0

for elm in data:
    data_in = (elm["pose"]["translation"]["y"] * (39.37008))
    x_data += str(data_in) + ","
    x_size += 1

x_head = "const std::vector<double> KaADAS_l_" + working_name + "_X" + " = {"
x_foot = "};"

x_output = x_head + x_data + x_foot

#y
y_data = ""
y_size = 0

for elm in data:
    data_in = (elm["pose"]["translation"]["x"] * (39.37008))
    y_data += str(data_in) + ","
    y_size += 1

y_head = "const std::vector<double> KaADAS_l_" + working_name + "_Y" + " = {"
y_foot = "};"

y_output = y_head + y_data + y_foot

with open(output_path + output_name, "w") as f:
    print("#include <vector>\n\n\n", file=f)
    print(time_output, file=f)
    print(rot_time_output, file=f)
    print(rot_output, file=f)
    print(x_output, file=f)
    print(y_output, file=f)
