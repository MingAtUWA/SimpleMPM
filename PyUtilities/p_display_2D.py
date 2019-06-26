import os
import numpy as np
import matplotlib.pyplot as plt

import oned_consolidation as oc

res_file_name = "..\\Build\\Tests\\res_file\\TimeHistory1-test_out1.txt"
#res_file_name = "..\\Build\\Tests\\res_file\\TimeHistory1-test_out1_no_damp.txt"

script_dir =  os.path.dirname(os.path.realpath('__file__')) #<-- absolute dir the script is in
abs_file_path = os.path.join(script_dir + "\\", res_file_name)

fig = plt.figure()
plot1 = fig.subplots(1, 1)
plot1.set_xlabel("Time (t)")
plot1.set_ylabel("Pore Pressure (Pa)")

#################################################################################################
x_data = [];
y_data = [];
time = 0.0
z = 0.0 # position
field_value = 0.0
output_pcl_num = 0
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
                if i == 1: # particle at bottom (impermeable)
                #if i == output_pcl_num - 1: # particle on top (free draining)
                    field_value = line_data[3] # p
                    y_data.append(field_value)
                    if not is_init:
                        z = line_data[1] # position
                        is_init = True

#plot1.set_xlim([0.0, 110.0])
#plt.xticks(np.arange(0.0, 110.0 + 1.0, 10.0))
#plt.yticks(np.arange(0.0, max(y_data) + 0.5, 0.5))

line1, = plot1.plot(x_data, y_data)


#################################################################################################
E = 1000.0
niu = 0.25 # possion ratio
Es = (1 - niu) / (1 + niu) / (1 - 2.0*niu) * E # Es = (1-v) / (1 + v) / (1-2v) * E
kv = 1.0e-4
miu = 1.0 # dynamic viscosity
Cv = kv * Es / miu
u0 = 10.0
H = 1.0
con_res = oc.OneDConsolidation(Cv, Es, u0, H)

time = 30.0 # time of consolidation
data_num = 100
t_list = np.zeros(data_num + 2)
u_list = np.zeros(data_num + 2)
t_list[0] = 0.0
u_list[0] = u0 * 166666.67 / (166666.67 + 120)
t_list[1] = 10.0
u_list[1] = u_list[0]
for i in range(data_num):
    t_list[i + 2] = time * float(i) / float(data_num)
    u_list[i + 2] = con_res.calPorePressure(t_list[i + 2], 1.0 - z)
    t_list[i + 2] += t_list[1]

line2, = plot1.plot(t_list, u_list, 'r--')

plt.legend(handles=[line1,line2], labels=['Explicit MPM', 'Analytical Solution'])

plt.show()
#os.system("pause")
