from csv_reader import *
from define import *

for i in range(1, 100):
    filename = "/home/zilin/Documents/python_file/racing_data/2_time_trial_data/bitmatta/flight_%d_pilot_bitmatta.csv" % (i)

    pilot = read_human_csv(filename)

    pilot.plot_2D()

plt.show()