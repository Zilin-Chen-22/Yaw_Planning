# from math import pi
import math
import matplotlib.pyplot as plt

class Gate:
    def __init__(self, x, y, z, angle,size):
        self.x = x
        self.y = y
        self.z = z
        self.angle = (angle - 90) * math.pi / 180
        self.size = size     # unit: m
        self.edge_1_x = self.x + self.size / 2 * math.cos(self.angle)
        self.edge_2_x = self.x - self.size / 2 * math.cos(self.angle)
        self.edge_1_y = self.y + self.size / 2 * math.sin(self.angle)
        self.edge_2_y = self.y - self.size / 2 * math.sin(self.angle)

class Traj:
    
    def __init__(self, time, p_x, p_y, p_z, gates):
        self.time = time
        self.p_x = p_x
        self.p_y = p_y
        self.p_z = p_z
        self.gates = gates
        self.passGateTime = []
        self.switchTime = []

        gates_count = 0

        for t in range(1, len(self.time)):
            l1x1 = self.p_x[t - 1]
            l1y1 = self.p_y[t - 1]
            l1x2 = self.p_x[t]
            l1y2 = self.p_y[t]
            l2x1 = self.gates[gates_count].edge_1_x
            l2y1 = self.gates[gates_count].edge_1_y
            l2x2 = self.gates[gates_count].edge_2_x
            l2y2 = self.gates[gates_count].edge_2_y

            if (max(l1x1, l1x2) < min(l2x1, l2x2)
                or max(l1y1, l1y2) < min(l2y1, l2y2)
                or max(l2x1, l2x2) < min(l1x1, l1x2)
                or max(l2y1, l2y2) < min(l1y1, l1y2)):
                continue
            elif (((l1x1 - l2x1) * (l2y2 - l2y1) - (l1y1 - l2y1) * (l2x2 - l2x1)) * 
                  ((l1x2 - l2x1) * (l2y2 - l2y1) - (l1y2 - l2y1) * (l2x2 - l2x1)) <= 0 or
                  ((l2x1 - l1x1) * (l1y2 - l1y1) - (l2y1 - l1y1) * (l1x2 - l1x1)) *
                  ((l2x2 - l1x1) * (l1y2 - l1y1) - (l2y2 - l1y1) * (l1x2 - l1x1)) <= 0):
                self.passGateTime.append(t)
                if gates_count != len(gates) - 1:
                    self.switchTime.append(t - 100)
                gates_count += 1
                if gates_count >= len(self.gates):
                    break
        self.passGateTime.append(len(self.time))
        self.switchTime.append(len(self.time))
        if 0 in self.switchTime:
            self.switchTime.remove(0)

    def plot_2D(self):
        # first plot traj
        plt.plot(self.p_x, self.p_y)
        plt.title("MAP")

        # then plot the gates
        for i in range(len(self.gates)):
            plt.plot([self.gates[i].edge_1_x, self.gates[i].edge_2_x], [self.gates[i].edge_1_y, self.gates[i].edge_2_y], 'k')
        
        # show the point that the drone is flying across a gate
        for i in self.passGateTime:
            plt.scatter(self.p_x[i], self.p_y[i], color = 'r', s = 100)
        plt.show()