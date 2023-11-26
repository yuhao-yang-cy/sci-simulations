import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider

# define constant parameters
t_step = 0.01      # time steps
n_iter = 800       # number of time steps to run
total_time = n_iter * t_step

# set up initial conditions
z0, z_dot0, z_dotdot0 = 0, 0, 0

def controller(Kp, Kd, func):
    # main code: defines the 1D PD controller
    # require paremeters: Kp, Kd, and the desired trajectory 'func'
    # func: a Python function that returns the desired height at different times
    
    t = np.arange(0, total_time, t_step)

    z, z_dot, z_dotdot = np.zeros(n_iter), np.zeros(n_iter), np.zeros(n_iter)
    z_des, z_dot_des, z_dotdot_des = np.zeros(n_iter), np.zeros(n_iter), np.zeros(n_iter)

    z[0], z_dot[0], z_dotdot[0] = z0, z_dot0, z_dotdot0

    for i in range(1, n_iter):
        # getting values for the desired path
        z_des[i], z_dot_des[i], z_dotdot_des[i] = func(i*t_step)
        # using Euler method to extrapolate the new height and new velocity
        z[i] = z[i-1] + z_dot[i-1] * t_step
        z_dot[i] = z_dot[i-1] + z_dotdot[i-1] * t_step
        # finding the desired acceleration with PD controller algorithm
        z_dotdot[i] = z_dotdot_des[i] + Kd * (z_dot_des[i] - z_dot[i]) + Kp * (z_des[i] - z[i])

    return z_des, z

# simulation: rise to a given height and hover
def const_height_traj(t):
    return [2, 0, 0]

# simulation: a sine curve trajectory
def sine_traj(t):
    z_des = np.sin(t)
    z_dot_des = np.cos(t)
    z_dotdot_des = -np.sin(t)
    return [z_des, z_dot_des, z_dotdot_des]

# simulation: jumping and hovering at a series of heights
def step_jump_traj(t):
    if t < total_time * 0.25:
        return [2, 0, 0]
    elif t < total_time * 0.5:
        return [-1, 0, 0]
    elif t < total_time * 0.75:
        return [1, 0, 0]
    else:
        return [2, 0, 0]
    
# simulation: motion along some awkward curve
def awkward_traj(t):
    z_des = 0.5 * t**2 - 2 * t + 3 * np.sin(2*t)
    z_dot_des = t - 2 + 6 * np.cos(2*t)
    z_dotdot_des = 1 - 12 * np.sin(2*t)
    return [z_des, z_dot_des, z_dotdot_des]

t = np.arange(0, total_time, t_step)

# Define initial parameters
Kp = 0
Kd = 0

z_des1, z1 = controller(Kp, Kd, const_height_traj)
z_des2, z2 = controller(Kp, Kd, sine_traj)
z_des3, z3 = controller(Kp, Kd, step_jump_traj)
z_des4, z4 = controller(Kp, Kd, awkward_traj)

# Create the figure and the line that we will manipulate
plt.rcParams["figure.figsize"] = (12, 9)
fig, ax = plt.subplots(2, 2)
line11, = ax[0,0].plot(t, z_des1, label='Desired Trajectory')
line12, = ax[0,0].plot(t, z1, label='Actural Trajectory')
ax[0,0].set_xlim(0, total_time)
ax[0,0].set_ylim(-0.5, 3.5)
ax[0,0].set_ylabel('Height [m]')
ax[0,0].set_title('Test 1')

line21, = ax[0,1].plot(t, z_des2, label='Desired Trajectory')
line22, = ax[0,1].plot(t, z2, label='Actural Trajectory')
ax[0,1].set_xlim(0, total_time)
ax[0,1].set_ylim(-2, 2)
ax[0,1].set_title('Test 2')

line31, = ax[1,0].plot(t, z_des3, label='Desired Trajectory')
line32, = ax[1,0].plot(t, z3, label='Actural Trajectory')
ax[1,0].set_xlim(0, total_time)
ax[1,0].set_ylim(-3.5, 4)
ax[1,0].set_xlabel('Time [s]')
ax[1,0].set_ylabel('Height [m]')
ax[1,0].set_title('Test 3')

line41, = ax[1,1].plot(t, z_des4, label='Desired Trajectory')
line42, = ax[1,1].plot(t, z4, label='Actural Trajectory')
ax[1,1].set_xlim(0, total_time)
ax[1,1].set_ylim(-5, 7)
ax[1,1].set_xlabel('Time [s]')
ax[1,1].set_title('Test 4')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(bottom=0.25)

# horizontal slider to control Kp
axKp = fig.add_axes([0.15, 0.1, 0.6, 0.04])
Kp_slider = Slider(
    ax=axKp,
    label=r'$K_p$',
    valmin=0,
    valmax=80,
    valinit=Kp,
)

# horizontal slider to control Kp
axKd = fig.add_axes([0.15, 0.04, 0.6, 0.04])
Kd_slider = Slider(
    ax=axKd,
    label=r'$K_d$',
    valmin=0,
    valmax=20,
    valinit=Kd,
)

# The function to be called anytime a slider's value changes
def update(val):
    z_des1, z1 = controller(Kp_slider.val, Kd_slider.val, const_height_traj)
    z_des2, z2 = controller(Kp_slider.val, Kd_slider.val, sine_traj)
    z_des3, z3 = controller(Kp_slider.val, Kd_slider.val, step_jump_traj)
    z_des4, z4 = controller(Kp_slider.val, Kd_slider.val, awkward_traj)
    line12.set_ydata(z1)
    line22.set_ydata(z2)
    line32.set_ydata(z3)
    line42.set_ydata(z4)
    fig.canvas.draw_idle()

# update function with each slider
Kp_slider.on_changed(update)
Kd_slider.on_changed(update)

# create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.81, 0.07, 0.08, 0.05])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    Kp_slider.reset()
    Kd_slider.reset()

button.on_clicked(reset)
plt.show()