import csv
import yaml
import numpy as np

def transformationAmod(fileName):
    with open(fileName) as f:
        reader = csv.reader(f, delimiter=',')
        target_file = open("amod_micromobility.dpc.yaml", 'w')
        data = list(reader)
        target_file.write("implementations:\n")
        for index in range(len(data)):
                target_file.write("  "+data[index][0]+": \n")
                target_file.write("    f_max:\n")
                target_file.write("    " + "- " + "\""+ data[index][1] + " dimensionless\" \n")
                target_file.write("    r_min:\n")
                target_file.write("    " + "- " + "\"" + data[index][2] + " Nat\" \n")
                target_file.write("    " + "- " + "\"" + data[index][3] + " Nat\" \n")
                target_file.write("    " + "- " + "\"" + data[index][4] + " miles/hour\" \n")
                target_file.write("    " + "- " + "\"" + data[index][5] + " miles/hour\" \n")
                target_file.write("    " + "- " + "\"" + data[index][6] + " s\" \n")
                target_file.write("    " + "- " + "\"" + data[index][7] + " meter/s\" \n")
                target_file.write("    " + "- " + "\"" + data[index][8] + " meter/s\" \n")
                target_file.write("    " + "- " + "\"" + data[index][9] + " Nat\" \n")

def transformationLQG(fileName):
    with open(fileName) as f:
        reader = csv.reader(f, delimiter=',')
        target_file = open("lane_control.dpc.yaml", 'w')
        data = list(reader)
        target_file.write("implementations:\n")
        for index in range(len(data)):
                target_file.write("  "+data[index][0]+": \n")
                target_file.write("    f_max:\n")
                target_file.write("    " + "- " + "\""+ data[index][1] + " dimensionless\" \n")
                target_file.write("    r_min:\n")
                target_file.write("    " + "- " + "\"" + data[index][2] + " dimensionless\" \n")
                obs_info = 1.0 / float(data[index][3])
                target_file.write("    " + "- " + "\"" + str(obs_info) + " dimensionless\" \n")
                freq = 1.0 / float(data[index][4])
                target_file.write("    " + "- " + "\"" + str(freq) + " Hz\" \n")
                target_file.write("    " + "- " + "\"" + data[index][5] + " dimensionless\" \n")
                target_file.write("    " + "- " + "\"" + str(freq) + " Hz\" \n")


#transformationAmod("amod.csv")
transformationLQG("cat_lqg.csv")