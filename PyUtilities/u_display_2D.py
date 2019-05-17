import matplotlib.pyplot as plt
import numpy as np
import os

import oned_consolidation as oc

solution_path = "H:\\Files_Ming\\Code\\SimpleMPM\\Output\\res_file\\"
res_file_name = "TimeHistory1 - test_out1.txt"
output_pcl_num = 2 * 10 * 4

fig = plt.figure()
plot1 = fig.subplots(1, 1)

x_data = [];
y_data = [];

time = 0.0
field_value = 0.0
with open(solution_path + res_file_name, 'r') as res_file:
    while True:
        # time record
        line_text = res_file.readline()
        if not line_text:
            break
        line_data = list(map(lambda x: x.strip('\n'), line_text.split(',')))
        time = float(line_data[4])
        for i in range(output_pcl_num):
            line_text = res_file.readline()
            if i == output_pcl_num - 2: # node at the boundary
                # uy_s
                data = list(map(lambda x: float(x.strip('\n')), line_text.split(',')))
                field_value = data[1] - 0.975 # y
                #field_value = data[2] # p
        x_data.append(time)
        y_data.append(field_value)

plot1.set_xlim([0.0, 35.0])
plot1.set_xlabel("Time (s)")
plot1.set_ylabel("Settlement (m)")

#print(x_data)
#print(y_data)

line1, = plot1.plot(x_data, y_data)

E = 1000.0
niu = 0.25 # possion ratio
Es = (1 - niu) / (1 + niu) / (1 - 2.0*niu) * E # Es = (1-v) / (1 + v) / (1-2v) * E
kv = 1.0e-4
miu = 1.0 # dynamic viscosity
Cv = kv * Es / miu
u0 = 10.0
H = 1.0
con_res = oc.OneDConsolidation(Cv, Es, u0, H)

data_num = 31
t_list = np.zeros(data_num + 2)
u_list = np.zeros(data_num + 2)
t_list[0] = 0.0
u_list[0] = 0.0
t_list[1] = 4.99
u_list[0] = 0.0
for i in range(data_num):
    t_list[i + 2] = 1.0 * float(i)
    u_list[i + 2] = con_res.calSettlement(t_list[i + 2])
    t_list[i + 2] += 5

line2, = plot1.plot(t_list, u_list, 'r--')

plt.legend(handles=[line1,line2], labels=['Explicit MPM', 'Analytical Solution'])

plt.show()
#os.system("pause")
