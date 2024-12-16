import csv
from define import *

def read_csv_columns():
    res = [[],[],[],[],[],[],[]]

    with open('data.csv', 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)

        for row in reader:
            if len(row) >= 3:
                for i in range(7):
                    res[i].append(float(row[i]))

    return res

def read_human_csv(name_of_csv):  
    number_of_viriables = 9

    data = []

    for i in range(number_of_viriables):
        data.append([])
    
    with open(name_of_csv, 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)

        next(reader)    # to skip the first line

        for row in reader:
            for variables in range(number_of_viriables):
                data[variables].append(float(row[variables]))

    return HUMAN_TRAJ(data)