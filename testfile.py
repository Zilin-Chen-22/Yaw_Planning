import yaw_planning
import math

w0 = 0  # Starting point angular velocity in rad/s
theta0 = math.pi / 2 # Starting point yaw angle in rad, compared to x-axis
T = math.pi / 4 + 0.125 - 0.025 # Total Time

gate_x = 4  # Gate position
gate_y = 5

[alpha, beta, gamma, a0] = yaw_planning.p2p_plan(w0, theta0, T, gate_x, gate_y)
yaw_planning.plot_yaw(alpha, beta, gamma, a0, w0, theta0, T, gate_x, gate_y)