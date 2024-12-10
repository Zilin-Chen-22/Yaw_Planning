from casadi import *
import matplotlib.pyplot as plt
import define as map

# Suming rounds for approximate to an Intergral
ROUNDS = 2000   # default value: 2000 times
GATES_NUM = 0
MAX_TIME_ROUNDS = 0
# LOOPTIME = 1  # Unit: ms

# unit rad/s
W_MAX = 5  
W_MIN = -W_MAX
A_MAX = 10
A_MIN = -A_MAX

def p2p_plan(w0, theta0, traj, gate):
    global ROUNDS
    ROUNDS = len(traj.time)
    # a0 = A_MIN  
    opti = Opti()
    
    alpha = opti.variable()
    beta = opti.variable()
    gamma = opti.variable()
    
    # Assume the acceleration can change immediately at the starting point
    a0 = opti.variable()    # Starting point acceleration in rad/s^(-2)  
    
    # t = opti.variable()

    opti.minimize( f1_x(alpha, beta, gamma, a0, w0, theta0, traj, gate.x, gate.y) )

    for i in range(ROUNDS):
        t = traj.time[i]

        if t == 0:
            t = 0.00001

        opti.subject_to( g1_x(alpha, beta, gamma, a0, w0, t) )
        opti.subject_to( h1_x(alpha, beta, gamma, a0, t) )
        opti.subject_to( g2_x(alpha, beta, gamma, a0, w0, t) )
        opti.subject_to( h2_x(alpha, beta, gamma, a0, t) )

    T = traj.time[-1]

    dx = (gate.x - traj.p_x[-1])
    dy = (gate.y - traj.p_y[-1])
    if dx == 0 and dy == 0:
        dx = (gate.x - traj.p_x[-2])
        dy = (gate.y - traj.p_y[-2])
        last_target = atan2(dy, dx)
    elif dx == 0 and dy > 0:
        last_target = pi / 2
    elif dx == 0 and dy < 0:
        last_target = - pi / 2
    else:
        last_target = atan2(dy, dx)
    opti.subject_to(theta(alpha, beta, gamma, a0, w0, theta0, T) == last_target)

    opti.solver('ipopt')

    sol = opti.solve()

    return [sol.value(alpha), sol.value(beta), sol.value(gamma), sol.value(a0)]

def gates_plan(w0, theta0, traj, gate):
    global MAX_TIME_ROUNDS
    MAX_TIME_ROUNDS = len(traj.time)
    # a0 = A_MIN  
    opti = Opti()
    
    alpha = opti.variable()
    beta = opti.variable()
    gamma = opti.variable()
    
    # Assume the acceleration can change immediately at the starting point
    a0 = opti.variable()    # Starting point acceleration in rad/s^(-2)  
    
    # t = opti.variable()

    opti.minimize( f2_x(alpha, beta, gamma, a0, w0, theta0, traj, gate.x, gate.y) )

    for i in range(MAX_TIME_ROUNDS):
        t = traj.time[i]

        if t == 0:
            t = 0.00001

        opti.subject_to( g1_x(alpha, beta, gamma, a0, w0, t) )
        opti.subject_to( h1_x(alpha, beta, gamma, a0, t) )
        opti.subject_to( g2_x(alpha, beta, gamma, a0, w0, t) )
        opti.subject_to( h2_x(alpha, beta, gamma, a0, t) )

    # T = traj.time[-1]

    # dx = (gate.x - traj.p_x[-1])
    # dy = (gate.y - traj.p_y[-1])
    # if dx != 0 and dy != 0:
    #     last_target = atan2(dy, dx)
    # elif dx == 0 and dy != 0:
    #     last_target = pi * dy / abs(dy)
    # else:
    #     dx = (gate.x - traj.p_x[-2])
    #     dy = (gate.y - traj.p_y[-2])
    #     last_target = atan2(dy, dx)
    # opti.subject_to(theta(alpha, beta, gamma, a0, w0, theta0, T) == last_target)

    opti.solver('ipopt')

    sol = opti.solve()

    return [sol.value(alpha), sol.value(beta), sol.value(gamma), sol.value(a0)]

# Objective function
def f1_x(alpha, beta, gamma, a0, w0, theta0, traj, gate_x, gate_y):
    # MAX_TIME_ROUNDS = len(traj.time)

    res = 0
    for i in range(MAX_TIME_ROUNDS):
        dx = (gate_x - traj.p_x[i])
        dy = (gate_y - traj.p_y[i])
        if dx == 0 and dy == 0:
            dx = (gate_x - traj.p_x[i - 1])
            dy = (gate_y - traj.p_y[i - 1])
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) - pi / 2) ** 2
        elif dx == 0 and dy > 0:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) - pi / 2) ** 2
        elif dx == 0 and dy < 0:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) + pi / 2) ** 2
        else:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) - atan2(dy, dx)) ** 2
    return res

def f2_x(alpha, beta, gamma, a0, w0, theta0, traj, gate_x, gate_y):
    # MAX_TIME_ROUNDS = len(traj.time)

    res = 0
    for i in range(MAX_TIME_ROUNDS):
        dx = (gate_x - traj.p_x[i])
        dy = (gate_y - traj.p_y[i])
        if dx == 0 and dy == 0:
            dx = (gate_x - traj.p_x[i - 1])
            dy = (gate_y - traj.p_y[i - 1])
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) - pi / 2) ** 2
        elif dx == 0 and dy > 0:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) + pi / 2) ** 2
        elif dx == 0 and dy < 0:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) - pi / 2) ** 2
        else:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) - atan2(dy, dx)) ** 2
    return res

        


#Limitations
def g1_x(alpha, beta, gamma, a0, w0, t):
    return omega(alpha, beta, gamma, a0, w0, t) <= W_MAX

def h1_x(alpha, beta, gamma, a0, t):
    return acc(alpha, beta, gamma, a0, t) <= A_MAX

# def i1_x(alpha, beta, gamma, a0, w0, theta0, T):
#     res = []
#     for i in range(1, ROUNDS + 1):
#         res.append(theta(alpha, beta, gamma, a0, w0, theta0, i * T / ROUNDS) <= 2 * pi)
#     return res

def j1_x(t, T):
    return t <= T

def g2_x(alpha, beta, gamma, a0, w0, t):
    return omega(alpha, beta, gamma, a0, w0, t) >= W_MIN

def h2_x(alpha, beta, gamma, a0, t):
    return acc(alpha, beta, gamma, a0, t) >= A_MIN

# def i2_x(alpha, beta, gamma, a0, w0, theta0, T):
#     res = []
#     for i in range(1, ROUNDS + 1):
#         res.append(theta(alpha, beta, gamma, a0, w0, theta0, i * T / ROUNDS) >= 0)
#     return res

def j2_x(t, T):
    (T)
    return t >= 0

# Polynomul function
def theta(alpha, beta, gamma, a0, w0, theta0, t):
    res = alpha * (t ** 5) / 120 + beta * (t ** 4) / 24 + gamma * (t ** 3) / 6 + a0 * (t ** 2) / 2 + w0 * t + theta0
    return res

def omega(alpha, beta, gamma, a0, w0, t):
    res = alpha * (t ** 4) / 24 + beta * (t ** 3) / 6 + gamma * (t ** 2) / 2 + a0 * t + w0
    return res

def acc(alpha, beta, gamma, a0, t):
    res = alpha * (t ** 3) / 6 + beta * (t ** 2) / 2 + gamma * t + a0
    return res
    
def plot_yaw(alpha, beta, gamma, a0, w0, theta0, traj, gate):
    rounds = len(traj.time)

    print("alpha: ", alpha)
    print("beta: ", beta)
    print("gamma: ", gamma)
    print("a0: ", a0)
    print()

    x_axis = traj.time
    y_axis = []
    target = []
    path_t = []
    acc_axis = []
    angular_velocity = []
    map_x = []
    map_y = []
    path_t_last = -90

    for i in range(rounds):
        # x_axis.append(i * dt)
        y_axis.append(theta(alpha, beta, gamma, a0, w0, theta0, traj.time[i]) * 180 / pi)
        angular_velocity.append(omega(alpha, beta, gamma, a0, w0, traj.time[i]))
        acc_axis.append(acc(alpha, beta, gamma, a0, traj.time[i]))
        map_x.append(traj.p_x[i])
        map_y.append(traj.p_y[i])
        dx = (gate.x - traj.p_x[i])
        dy = (gate.y - traj.p_y[i])
        ds = sqrt(dx ** 2 + dy ** 2)
        if ds != 0:
            target.append(asin(dy / ds) * 180 / pi)
        else:
            target.append(target[-1])
        
        if i != rounds - 1:
            dx = (traj.p_x[i + 1] - traj.p_x[i])
            dy = (traj.p_y[i + 1] - traj.p_y[i])
            ds = sqrt(dx ** 2 + dy ** 2)
            if ds != 0:
                path_t_last = asin(dy / ds) * 180 / pi
            path_t.append(path_t_last)
        else:
            path_t.append(path_t_last)

    plt.subplot(221)
    plt.plot(x_axis, y_axis, 'r', label = "Planned Yaw")
    plt.plot(x_axis, target, 'g', label = "Target Angel")
    plt.plot(x_axis, path_t, 'b', label = "Original Yaw")
    plt.annotate('local max', xy=(2, 1), xytext=(3, 1.5))
    plt.title("Yaw Planning")
    plt.xlabel("Time(s)")
    plt.ylabel("Theta(degree)")
    plt.legend(loc = "best")

    plt.subplot(222)
    plt.plot(x_axis, angular_velocity)
    plt.title("Angular Velocity")
    plt.xlabel("Time(s)")
    plt.ylabel("w(rad/s)")

    plt.subplot(223)
    plt.plot(x_axis, acc_axis)
    plt.title("Acceleration")
    plt.xlabel("Time(s)")
    plt.ylabel("Acceleration(rad/s)")

    plt.subplot(224)
    plt.plot(map_x, map_y)
    plt.scatter(gate.x, gate.y, color = 'r', s = 100)
    plt.title("MAP")
