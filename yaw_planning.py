from casadi import *
import matplotlib.pyplot as plt

# Suming rounds for approximate to an Intergral
ROUNDS = 2000   # default value: 2000 times
LOOPTIME = 1  # Unit: ms

# unit rad/s
W_MAX = 5   
W_MIN = -W_MAX
A_MAX = 10
A_MIN = -A_MAX

def p2p_plan(w0, theta0, T, gate_x, gate_y):
    global ROUNDS
    ROUNDS = int(T * (10 ** 3) / LOOPTIME)
    # a0 = A_MIN  
    opti = Opti()

    alpha = opti.variable()
    beta = opti.variable()
    gamma = opti.variable()
    
    # Assume the acceleration can change immediately at the starting point
    a0 = opti.variable()    # Starting point acceleration in rad/s(-2)  
    
    # t = opti.variable()

    opti.minimize( f_x(alpha, beta, gamma, a0, w0, theta0, T, gate_x, gate_y) )

    for i in range(1, ROUNDS + 1):
        t = i * T / ROUNDS

        opti.subject_to( g1_x(alpha, beta, gamma, a0, w0, t))
        opti.subject_to( h1_x(alpha, beta, gamma, a0, t) )
        # opti.subject_to( W_MIN <= (alpha * (t ** 4) / 24 + beta * (t ** 3) / 6 + gamma * (t ** 2) / 2 + a0 * t + w0) <= W_MAX )
        # opti.subject_to( j1_x(t, T) )
        opti.subject_to( g2_x(alpha, beta, gamma, a0, w0, t) )
        opti.subject_to( h2_x(alpha, beta, gamma, a0, t) )
        # opti.subject_to( i2_x(alpha, beta, gamma, a0, w0, theta0, T) )
        # opti.subject_to( j2_x(t, T) )

    dx = (gate_x - x_pos(T))
    dy = (gate_y - y_pos(T))
    if dx == 0 and dy == 0:
        dx = (gate_x - x_pos(T - T / ROUNDS))
        dy = (gate_y - y_pos(T - T / ROUNDS))
    elif dx == 0 and dy > 0:
        last_target = pi / 2
    elif dx == 0 and dy < 0:
        last_target = - pi / 2
    else:
        last_target = atan(dy / dx)
    opti.subject_to( theta(alpha, beta, gamma, a0, w0, theta0, T) == last_target)

    opti.solver('ipopt')

    sol = opti.solve()

    return [sol.value(alpha), sol.value(beta), sol.value(gamma), sol.value(a0)]

# Objective function
def f_x(alpha, beta, gamma, a0, w0, theta0, T, gate_x, gate_y):
    res = 0
    dt = T / ROUNDS
    for i in range(1, ROUNDS + 1):
        dx = (gate_x - x_pos(i * dt))
        if dx == 0:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, i * dt) - pi / 2) ** 2
        else:
            res += (theta(alpha, beta, gamma, a0, w0, theta0, i * dt) - atan((gate_y - y_pos(i * dt)) / dx)) ** 2
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
    
def plot_yaw(alpha, beta, gamma, a0, w0, theta0, T, gate_x, gate_y):
    print("alpha: ", alpha)
    print("beta: ", beta)
    print("gamma: ", gamma)
    print("a0: ", a0)
    print()

    x_axis = []
    y_axis = []
    target = []
    path_t = []
    acc_axis = []
    angular_velocity = []
    map_x = []
    map_y = []
    for i in range(1, ROUNDS + 1):
        dt = T / ROUNDS
        x_axis.append(i * dt)
        y_axis.append(theta(alpha, beta, gamma, a0, w0, theta0, i * dt) * 180 / pi)
        angular_velocity.append(omega(alpha, beta, gamma, a0, w0, i * dt))
        acc_axis.append(acc(alpha, beta, gamma, a0, i * dt))
        map_x.append(x_pos(i * dt))
        map_y.append(y_pos(i * dt))
        dx = (gate_x - x_pos(i * dt))
        if dx == 0:
            target.append(90)
        else:
            target.append(atan((gate_y - y_pos(i * dt)) / dx) * 180 / pi)
        dx = (x_pos(i * dt) - x_pos((i - 1) * dt))
        if dx == 0:
            path_t.append(90)
        else:
            path_t.append(atan((y_pos(i * dt) - y_pos((i - 1) * dt)) / dx) * 180 / pi)

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
    plt.scatter(gate_x, gate_y, color = 'r', s = 100)
    plt.title("MAP")

    plt.show()

# Trajectory
def x_pos(t):
    if t < 0.125:
        return 0
    else:
        return 4 - 4 * cos((t - 0.125) * 2)

def y_pos(t):
    if t < 0.125:
        return t / 8
    else:
        return 4 * sin((t - 0.125) * 2) + 1