import matplotlib.pyplot as plt
import numpy as np
import os

from bar_axial_vibration import BarAxialVibration

#####################################################
# For purely mechanical problem 
#####################################################

res_file_name = "..\\Build\\Tests\\res_file\\TimeHistory1-test_out1.txt"

script_dir =  os.path.dirname(os.path.realpath('__file__')) #<-- absolute dir the script is in
abs_file_path = os.path.join(script_dir + "\\", res_file_name)

fig = plt.figure()
plot1 = fig.subplots(1, 1)
plot1.set_xlabel("Time (s)")
plot1.set_ylabel("Displacement (m)")

#######################################################################################
x_data = [];
y_data = [];
time = 0.0
output_pcl_num = 0
field_value = 0.0
is_init = False
with open(abs_file_path, 'r') as res_file:
    while True:
        # time record
        line_text = res_file.readline()
        if not line_text:
            break
        if ("TotalTime" in line_text):
            data_start_pos = line_text.find("=") + 1
            time = float(line_text[data_start_pos:-1])
            x_data.append(time)
        elif ("PointNum" in line_text):
            data_start_pos = line_text.find("=") + 1
            output_pcl_num = int(line_text[data_start_pos:-1])
        elif ("*FieldData" in line_text):
            for i in range(output_pcl_num):
                line_text = res_file.readline()
                line_data = list(map(lambda x: float(x.strip('\n')), line_text.split(',')))
                if i == output_pcl_num - 1: # particle on top
                    field_value = line_data[1] # y
                    if not is_init:
                        z = line_data[1] # y position
                        is_init = True
                    field_value -= z
                    y_data.append(field_value)

line1, = plot1.plot(x_data, y_data)

#################################################################################################
data_num = len(x_data)
ana_solution = np.zeros(data_num)
H = 1.0
p0 = -1.0
bf = 0.0
E = 1000.0
niu = 0.25
density = 10.0
E = (1 - niu) / (1 + niu) / (1 - 2.0*niu) * E # Es = (1-v) / (1 + v) / (1-2v) * E
bav = BarAxialVibration(H, p0, bf, E, density)
for i in range(data_num):
    ana_solution[i] = bav.displacement(H, x_data[i])
line2, = plot1.plot(x_data, ana_solution, '--k')

################################################################################################
plt.legend(handles=[line1,line2], labels=['Explicit MPM', 'Analytical Solution'])
plt.show()