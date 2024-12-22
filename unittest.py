from csv_reader import *
from define import *
import matplotlib
matplotlib.use('TkAgg')

for i in range(1, 100):
    filename = "../racing_data/2_time_trial_data/bitmatta/flight_%d_pilot_bitmatta.csv" % (i)

    pilot = read_human_csv(filename)

    pilot.plot_2D()

plt.show(block = False)

input("Press Enter to close all the figures ...... ")

plt.close("all")