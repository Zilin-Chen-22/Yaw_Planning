import yaw_planning
import math
import define as map
import matplotlib.pyplot as plt
import csv_reader as csv

planner = 1

data_set = csv.read_csv_columns()

w0 = 0 # Starting point angular velocity in rad/s
theta0 = 0 # Starting point yaw angle in rad, compared to x-axis
# T = math.pi / 4 + 0.125 - 0.125 # Total Time

traj_t = data_set[0]
traj_x = data_set[1]
traj_y = data_set[2]
qw = data_set[3]
qx = data_set[4]
qy = data_set[5]
qz = data_set[6]
human_yaw = []
human_pitch = []
human_roll = []

for i in range(len(qw)):
    a1 = qw[i] * qx[i] + qy[i] * qz[i]
    a2 = 1 - 2 * (qx[i] * qx[i] + qy[i] * qy[i])
    human_roll.append(math.atan2(2 * a1, a2) * 180 / math.pi)
    human_pitch.append(math.asin(2 * (qw[i] * qy[i] - qz[i] * qx[i])) * 180 / math.pi)
    human_yaw.append(math.atan2(qw[i] * qz[i] - qx[i] * qy[i], 1 - 2 * (qy[i] * qy[i] + qz[i] * qz[i])) * 180 / math.pi)
    if i >= 2:
        if human_yaw[-1] - human_yaw[-2] > 180:
            human_yaw[-1] -= 360
        elif human_yaw[-1] - human_yaw[-2] < -180:
            human_yaw[-1] += 360

gates_pos = [[-1.1, -1.6, 3.6, 0, 2.4], [9.2, 6.6, 1, -20, 2.4], [9.2, -4.0, 1.2, -130, 2.0], [-4.5, -6, 3.5, 180, 2.4], [-4.5, -6.0, 0.8, 0, 2.4], [4.75, -0.9, 1.2, 70, 3], [-2.8, 6.8, 1.2, -135, 2.4]]
gates = []
for i in gates_pos:
    gates.append(map.Gate(i[0], i[1], i[2], i[3], i[4]))

end = map.Gate(traj_x[-1], traj_y[-1], 0, 0, 0.1)
gates.append(end)

traj = map.Traj(traj_t, traj_x, traj_y, [], gates)

if planner:
    alpha_all = []
    beta_all = []
    gamma_all = []
    theta0_all = []
    w0_all = []
    a0_all = []
    time_stamp = []

    for i in range(len(gates)):
        # print(traj.switchTime)
        # plt.figure(i + 1)
        time_2 = []
        if i != 0:
            start_point = traj.switchTime[i - 1]
        else:
            start_point = 0
        end_point = traj.switchTime[i]
        time_t = traj.time[start_point : end_point]
        for j in range(len(time_t)):
            time_2.append(time_t[j] - time_t[0])

        traj_02 = map.Traj(time_2, traj.p_x[start_point : end_point], 
                        traj.p_y[start_point: end_point], [], gates)
        [alpha, beta, gamma, a0] = yaw_planning.gates_plan(w0, theta0, traj_02, gates[i])
        # yaw_planning.plot_yaw(alpha, beta, gamma, a0, w0, theta0, traj_02, gates[i])

        alpha_all.append(alpha)
        beta_all.append(beta)
        gamma_all.append(gamma)
        a0_all.append(a0)
        theta0_all.append(theta0)
        w0_all.append(w0)
        time_stamp += time_2

        w_new =  yaw_planning.omega(alpha, beta, gamma, a0, w0, traj_02.time[-1])
        theta_new = yaw_planning.theta(alpha, beta, gamma, a0, w0, theta0, traj_02.time[-1])

        w0 = w_new
        theta0 = theta_new


planned_theta = []
target_theta = []
target_theta_with_SP = []
tangent_theta = []
path_t_last = 0
human = human_yaw

gates_count = 0
target_gates_count = 0
for i in range(len(traj.time)):
    if planner:
        if i >= traj.switchTime[gates_count] and gates_count < len(gates):
            gates_count += 1
        if i >= traj.passGateTime[gates_count] and target_gates_count < len(gates):
            target_gates_count += 1
        planned_theta.append(yaw_planning.theta(alpha_all[gates_count], beta_all[gates_count], gamma_all[gates_count], 
                                                a0_all[gates_count], w0_all[gates_count], theta0_all[gates_count], time_stamp[i]) * 180 / math.pi)
    dx = (gates[target_gates_count].x - traj.p_x[i])
    dy = (gates[target_gates_count].y - traj.p_y[i])
    if dx != 0 and dy != 0:
        target_theta.append(math.atan2(dy, dx) * 180 / math.pi)
    elif dx == 0 and dy != 0:
        target_theta.append(90 * dy / abs(dy))
    else:
        target_theta.append(target_theta[-1])
    if i >= 2:
        if target_theta[-1] - target_theta[-2] > 180:
            target_theta[-1] -= 360
        elif target_theta[-1] - target_theta[-2] < -180:
            target_theta[-1] += 360
    
    dx = (gates[gates_count].x - traj.p_x[i])
    dy = (gates[gates_count].y - traj.p_y[i])
    ds = math.sqrt(dx ** 2 + dy ** 2)
    if dx != 0 and dy != 0:
        target_theta_with_SP.append(math.atan2(dy, dx) * 180 / math.pi)
    elif dx == 0 and dy != 0:
        target_theta_with_SP.append(90 * dy / abs(dy))
    else:
        target_theta_with_SP.append(target_theta_with_SP[-1])
    if i >= 2:
        if target_theta_with_SP[-1] - target_theta_with_SP[-2] > 180:
            target_theta_with_SP[-1] -= 360
        elif target_theta_with_SP[-1] - target_theta_with_SP[-2] < -180:
            target_theta_with_SP[-1] += 360
        
    
    # if i != len(traj.time) - 1:
    #     dx = (traj.p_x[i + 1] - traj.p_x[i])
    #     dy = (traj.p_y[i + 1] - traj.p_y[i])
    #     ds = math.sqrt(dx ** 2 + dy ** 2)
    #     if ds != 0:
    #         path_t_last = math.asin(dy / ds) * 180 / math.pi
    #     tangent_theta.append(path_t_last)
    # else:
    #     tangent_theta.append(path_t_last)

plt.figure(len(gates) + 1)
# plt.subplot(221)
if planner:
    plt.plot(traj.time, planned_theta, 'r', label = "Planned Yaw")
    plt.plot(traj.time, target_theta_with_SP, 'k', label = "Target Angle(With SP)")
plt.plot(traj.time, target_theta, 'g', label = "Target Angle(Without SP)")
plt.plot(traj.time, human, 'b', label = "Human Pilot")
plt.title("Yaw Planning")
plt.xlabel("Time(s)")
plt.ylabel("Theta(degree)")
plt.legend(loc = "best")

# plt.subplot(222)
plt.figure(len(gates) + 2)
plt.plot(traj.p_x, traj.p_y)
for i in range(len(traj.gates)):
    plt.plot([traj.gates[i].edge_1_x, traj.gates[i].edge_2_x], [traj.gates[i].edge_1_y, traj.gates[i].edge_2_y], 'k')

arrow_lenth = 1

for i in range(0, len(human), 25):
    arrow_x = [traj_x[i], traj_x[i] + arrow_lenth * math.cos(human[i] * math.pi / 180)]
    arrow_y = [traj_y[i], traj_y[i] + arrow_lenth * math.sin(human[i] * math.pi / 180)]
    plt.plot(arrow_x, arrow_y, color = 'b', lw = 1) 
    plt.annotate('', xy = (arrow_x[1], arrow_y[1]), xytext = (arrow_x[0], arrow_y[0]), arrowprops = dict(arrowstyle = '->', lw = 2, color = 'b'))
    if planner:
        arrow_x = [traj_x[i], traj_x[i] + arrow_lenth * math.cos(planned_theta[i] * math.pi / 180)]
        arrow_y = [traj_y[i], traj_y[i] + arrow_lenth * math.sin(planned_theta[i] * math.pi / 180)]
        plt.plot(arrow_x, arrow_y, color = 'r', lw = 1) 
        plt.annotate('', xy = (arrow_x[1], arrow_y[1]), xytext = (arrow_x[0], arrow_y[0]), arrowprops = dict(arrowstyle = '->', lw = 2, color = 'r'))
        # arrow_x = [traj_x[i], traj_x[i] + arrow_lenth * math.cos(target_theta_with_SP[i] * math.pi / 180)]
        # arrow_y = [traj_y[i], traj_y[i] + arrow_lenth * math.sin(target_theta_with_SP[i] * math.pi / 180)]
        # plt.plot(arrow_x, arrow_y, color = 'k', lw = 1) 
        # plt.annotate('', xy = (arrow_x[1], arrow_y[1]), xytext = (arrow_x[0], arrow_y[0]), arrowprops = dict(arrowstyle = '->', lw = 2, color = 'k'))

plt.show()
