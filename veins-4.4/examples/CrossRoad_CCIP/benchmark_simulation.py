#!/usr/bin/env python

from multiprocessing import Process

import os
import io
import subprocess
import shutil

print("Begin benchmark simulation")
print(os.getcwd())

current = os.getcwd()
run = current + "\\run"
results = current + "\\results"

config_xml = "paramics.launchd.xml"


# Leer archivo de configuracion
line_pos = 0
all_file = []
networks = []
with io.open(config_xml, 'r') as f:
    for line in f:
        all_file.append(line)
        if "network" in line and "ejemplo" in line:
            networks.append([line, line_pos])

        line_pos += 1


# Cambiar a siguiente simulacion
actual_network = []
next_network = []
for i in range(len(networks)):
    l = networks[i]

    if "<!--" not in l[0]:
        actual_network = l

        next_network = networks[(i + 1) % len(networks)]

print(actual_network)
print(next_network)
print("change to")

actual_network[0] = actual_network[0][:1] + "<!--" + actual_network[0][1:-1] + "-->" + actual_network[0][-1]
next_network[0] = next_network[0][:1] + next_network[0][5:-4] + next_network[0][-1]
print(actual_network)
print(next_network)

all_file[actual_network[1]] = actual_network[0]
all_file[next_network[1]] = next_network[0]

# Guardar resultados de simulacion anterior
if len(os.listdir(results)) > 0:
    print("Saving last simulation")
    last_net = actual_network[0]

    ini = last_net.find("\"") + 1
    end = last_net[ini:].find("\"")
    last_net = (last_net[ini:])[:end]
    
    save_res = current + "\\benchmark_results\\" + last_net

    try:
        os.makedirs(save_res)
        print("Directory " , save_res ,  " Created ") 
    except:
        print("Directory " , save_res ,  " already exists, simulation alredy made")

    for res in os.listdir(results):
        res_path = results + "\\" + res

        shutil.move(res_path, save_res)
else:
    print("No simulation to save")


# Modificar archivo de configuracion para siguiente simulacion
with io.open(config_xml, 'w') as f:
    for line in all_file:
        f.write(line)


# Iniciar batch de simulacion
p = subprocess.Popen( ['bash', '-c', './run -u Cmdenv -c nodebug'], shell=False)
p.wait()
