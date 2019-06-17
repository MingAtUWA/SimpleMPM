import matplotlib.pyplot as plt
import numpy as np
import math
import os

res_file_name = "..\\Build\\Tests\\res_file\\TimeHistory1-test_out1.txt"
#res_file_name = "..\\Build\\Tests\\res_file\\TimeHistory1-test_out1_no_damp.txt"

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
field_value = 0.0
output_pcl_num = 0
z = 0.0
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
                if i == output_pcl_num - 1: # node at the boundary
                #if i == 0:
                    field_value = line_data[0] # x
                    if (is_init == False):
                        is_init = True
                        z = field_value
                    field_value -= z
                    y_data.append(field_value)

line1, = plot1.plot(x_data, y_data)

#######################################################################################
F = 0.01
E = 100.0
density = 20.0
bar_len = 1.0
pcl_len = bar_len / output_pcl_num
w = math.sqrt(E / (density * bar_len))
bar_len -= pcl_len
x_amp = F * bar_len / E * 0.925

time = 5.0
data_num = 100
dt = time / float(data_num)
t_list = np.zeros(data_num+1)
u_list = np.zeros(data_num+1)
for i in range(data_num+1):
    t_list[i] = dt * float(i)
    u_list[i] = x_amp * math.cos(w * t_list[i]) - x_amp

#line2, = plot1.plot(t_list, u_list, 'r--')

#plt.legend(handles=[line1,line2], labels=['Explicit MPM', 'Analytical Solution'])

plt.show()
#plt.savefig('ut - 100 - 2e-5.png')

#os.system("pause")
