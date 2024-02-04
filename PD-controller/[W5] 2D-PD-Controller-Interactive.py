import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider

# define physical constants
mass = 0.2       # mass of the UAV
Ixx = 0.008      # moment of inertia
g = 9.81         # acceleration of free fall

# set up initial conditions
y0, y_dot0, y_dotdot0 = 0, 0, 0
z0, z_dot0, z_dotdot0 = 0, 0, 0
phi0 = 0, 0

# main function: PD controller
# required parameters: Kp and Kd parameters for y, z and phi
# 'func' is a Python function that describes the desired trajectory for the UAV to follow
def controller(K, t_step, n_iter, func):
    Kp_y, Kd_y, Kp_z, Kd_z, Kp_phi, Kd_phi = K
    t = np.arange(0.0, n_iter*t_step, t_step)
    # setting up the arrays to store desired trajectory and actual trajectory
    y, y_dot, y_dotdot = np.zeros(n_iter), np.zeros(n_iter), np.zeros(n_iter)
    y_des, y_dot_des, y_dotdot_des = np.zeros(n_iter), np.zeros(n_iter), np.zeros(n_iter)
    z, z_dot, z_dotdot = np.zeros(n_iter), np.zeros(n_iter), np.zeros(n_iter)
    z_des, z_dot_des, z_dotdot_des = np.zeros(n_iter), np.zeros(n_iter), np.zeros(n_iter)
    phi, phi_dot, phi_dotdot = np.zeros(n_iter), np.zeros(n_iter), np.zeros(n_iter)
    phi_des = np.zeros(n_iter)
    
    # initialising the arrays to store the required thrust and torque
    F = np.zeros(n_iter)
    tau = np.zeros(n_iter)

    # setting up the initial values for actual trajectory
    y[0], y_dot[0], y_dotdot[0] = y0, y_dot0, y_dotdot0
    z[0], z_dot[0], z_dotdot[0] = z0, z_dot0, z_dotdot0
    F[0] = mass * g

    # feed in coordinates, velocities for desired trajectory
    for i in range(0, n_iter): 
        y_des[i], y_dot_des[i], y_dotdot_des[i] = func(i*t_step)[0]
        z_des[i], z_dot_des[i], z_dotdot_des[i] = func(i*t_step)[1]

    for i in range(1, n_iter):    
        # equations of motion for quadrotor in y, z and phi
        y_dotdot[i] = -F[i-1]/mass * np.sin(phi[i-1])
        z_dotdot[i] = F[i-1]/mass * np.cos(phi[i-1]) - g
        phi_dotdot[i] = tau[i-1] / Ixx
    
        # estimate actual trajectory using difference method
        y[i] = y[i-1] + y_dot[i-1] * t_step
        y_dot[i] = y_dot[i-1] + y_dotdot[i-1] * t_step
        z[i] = z[i-1] + z_dot[i-1] * t_step
        z_dot[i] = z_dot[i-1] + z_dotdot[i-1] * t_step
        phi[i] = phi[i-1] + phi_dot[i-1] * t_step
        phi_dot[i] = phi_dot[i-1] + phi_dotdot[i-1] * t_step
    
        # compute the required output from PD controller
        phi_des[i] = - (y_dotdot_des[i] + Kd_y * (y_dot_des[i] - y_dot[i]) + Kp_y * (y_des[i] - y[i])) / g
        F[i] = mass * (g + z_dotdot_des[i] + Kd_z * (z_dot_des[i] - z_dot[i]) + Kp_z * (z_des[i] - z[i]) )
        tau[i] = Ixx * (-Kd_phi * phi_dot[i] + Kp_phi * (phi_des[i] - phi[i]))

    return y_des, z_des, y, z

# define constant parameters
t_step = 0.01
n_iter = 800
total_time = t_step * n_iter

# test 1: horizontal motion at a fixed height
def const_height_traj(t):
    # return values [[y_des, vy_des, ay_des], [z_des, vz_des, az_des]]
    return [[t, 1, 0], [1, 0, 0]]

# test 2: fly along a sinusoidal path
def sine_traj(t):
    # return values [[y_des, vy_des, ay_des], [z_des, vz_des, az_des]]
    return [[t, 1, 0], [np.sin(t), np.cos(t), -np.sin(t)]]

t = np.arange(0, total_time, t_step)

# initial PD parameters
Kp_y = 0
Kd_y = 0
Kp_z = 0
Kd_z = 0
Kp_phi = 0
Kd_phi = 0
K = Kp_y, Kd_y, Kp_z, Kd_z, Kp_phi, Kd_phi

y_des1, z_des1, y1, z1 = controller(K, t_step, n_iter, const_height_traj)
y_des2, z_des2, y2, z2 = controller(K, t_step, n_iter, sine_traj)

# Create the figure and the line that we will manipulate
plt.rcParams["figure.figsize"] = (12, 9)
fig, ax = plt.subplots(1,2)
line11, = ax[0].plot(y_des1, z_des1, label='Desired Trajectory')
line12, = ax[0].plot(y1, z1, label='Actural Trajectory')
ax[0].set_xlabel('Horizontal Distance [m]')
ax[0].set_ylabel('Height [m]')
ax[0].set_ylim(-1, 3)
ax[0].set_title('Test 1')

line21, = ax[1].plot(y_des2, z_des2, label='Desired Trajectory')
line22, = ax[1].plot(y2, z2, label='Actural Trajectory')
ax[1].set_xlabel('Horizontal Distance [m]')
ax[1].set_ylabel('Height [m]')
ax[1].set_ylim(-2, 2)
ax[1].set_title('Test 2')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(bottom=0.5)

# horizontal slider to control Kp_y
axKpy = fig.add_axes([0.152, 0.35, 0.7, 0.03])
Kpy_slider = Slider(
    ax=axKpy,
    label=r'$K_{p,y}$',
    valmin=0,
    valmax=80,
    valinit=Kp_y,
)

# horizontal slider to control Kd_y
axKdy = fig.add_axes([0.15, 0.3, 0.7, 0.03])
Kdy_slider = Slider(
    ax=axKdy,
    label=r'$K_{d,y}$',
    valmin=0,
    valmax=30,
    valinit=Kd_y,
)

# horizontal slider to control Kp_z
axKpz = fig.add_axes([0.152, 0.25, 0.7, 0.03])
Kpz_slider = Slider(
    ax=axKpz,
    label=r'$K_{p,z}$',
    valmin=0,
    valmax=80,
    valinit=Kp_z,
)

# horizontal slider to control Kd_z
axKdz = fig.add_axes([0.15, 0.2, 0.7, 0.03])
Kdz_slider = Slider(
    ax=axKdz,
    label=r'$K_{d,z}$',
    valmin=0,
    valmax=30,
    valinit=Kd_z,
)

# horizontal slider to control Kp_z
axKpphi = fig.add_axes([0.152, 0.15, 0.7, 0.03])
Kpphi_slider = Slider(
    ax=axKpphi,
    label=r'$K_{p,\phi}$',
    valmin=0,
    valmax=2000,
    valinit=Kp_phi,
)

# horizontal slider to control Kd_z
axKdphi = fig.add_axes([0.15, 0.1, 0.7, 0.03])
Kdphi_slider = Slider(
    ax=axKdphi,
    label=r'$K_{d,\phi}$',
    valmin=0,
    valmax=80,
    valinit=Kd_phi,
)

# The function to be called anytime a slider's value changes
def update(val):
    K = Kpy_slider.val, Kdy_slider.val, Kpz_slider.val, Kdz_slider.val, Kpphi_slider.val, Kdphi_slider.val
    y_des1, z_des1, y1, z1 = controller(K, t_step, n_iter, const_height_traj)
    y_des2, z_des2, y2, z2 = controller(K, t_step, n_iter, sine_traj)
    line12.set_xdata(y1)
    line12.set_ydata(z1)
    line22.set_xdata(y2)
    line22.set_ydata(z2)
    fig.canvas.draw_idle()

# update function with each slider
Kpy_slider.on_changed(update)
Kdy_slider.on_changed(update)
Kpz_slider.on_changed(update)
Kdz_slider.on_changed(update)
Kpphi_slider.on_changed(update)
Kdphi_slider.on_changed(update)

# create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.45, 0.04, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    Kpy_slider.reset()
    Kdy_slider.reset()
    Kpz_slider.reset()
    Kdz_slider.reset()
    Kpphi_slider.reset()
    Kdphi_slider.reset()

button.on_clicked(reset)
plt.show()