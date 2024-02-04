import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider

t_min = 0
t_max = 8
n = 800
dt = (t_max - t_min)/n

t = np.linspace(t_min, t_max, n)

# numerical solution with the Runge–Kutta method
def damped_rk(beta, omega, x0, v0):
    # setting up x/v vectors and imposing initial conditions
    x = np.zeros_like(t)
    v = np.zeros_like(t)
    x[0] = x0
    v[0] = v0

    # looping for all time slices
    # implementation of four-point approximation
    for i in range(1, len(x)):
        v1 = v[i-1]
        x1 = x[i-1]
        a1 = - omega** 2 * x1 - 2 * beta * v1

        v2 = v1 + a1 * dt/2
        a2 = - omega**2 * (x1 + v1*dt/2) - 2 * beta * (v2)
        
        v3 = v1 + a2*dt/2
        a3 = - omega**2 * (x1 + v2*dt/2) - 2 * beta * (v3)
        
        v4 = v1 + a3*dt
        a4 = - omega**2 * (x1 + v3*dt) - 2 * beta * (v4)
        
        dv = (a1 + 2*a2 + 2*a3 + a4) * dt / 6
        dx = (v1 + 2*v2 + 2*v3 + v4) * dt / 6

        v[i] = v[i-1] + dv
        x[i] = x[i-1] + dx

    return x

# default settings for the plot
omega = 10
beta = 0
x0 = 0
v0 = 0
x = damped_rk(beta, omega, x0, v0)

# create the figure
plt.rcParams["figure.figsize"] = (8, 6)
fig, ax = plt.subplots()
line, = ax.plot(t, x, color='green')
ax.set_xlabel('time $t$ (s)')
ax.set_ylabel('displacement $x$ (cm)')
ax.set_ylim(-6, 6)
plt.grid(linestyle = '--', linewidth = 0.5)
plt.title(r'Damped Oscillation Simulation')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(bottom=0.33)

# horizontal sliders for interactive controls
axbeta = fig.add_axes([0.2, 0.15, 0.5, 0.04])
beta_slider = Slider(
    ax=axbeta,
    label=r'$\beta$',
    valmin=0,
    valmax=10,
    valinit=beta,
)

axomega = fig.add_axes([0.2, 0.2, 0.5, 0.04])
omega_slider = Slider(
    ax=axomega,
    label=r'$\omega$',
    valmin=0,
    valmax=25,
    valinit=omega,
)

axx0 = fig.add_axes([0.2, 0.1, 0.5, 0.04])
x0_slider = Slider(
    ax=axx0,
    label=r'$x_0$',
    valmin=-3,
    valmax=3,
    valinit=x0,
)

axv0 = fig.add_axes([0.2, 0.05, 0.5, 0.04])
v0_slider = Slider(
    ax=axv0,
    label=r'$v_0$',
    valmin=-50,
    valmax=50,
    valinit=v0,
)

# update the plot when any slider value changes
def update(val):
    x = damped_rk(beta_slider.val, omega_slider.val, x0_slider.val, v0_slider.val)
    line.set_ydata(x)
    fig.canvas.draw_idle()

# register update function with each slider
beta_slider.on_changed(update)
omega_slider.on_changed(update)
x0_slider.on_changed(update)
v0_slider.on_changed(update)

# create a reset button to set all sliders to initial values
resetax = fig.add_axes([0.8, 0.125, 0.08, 0.08])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    beta_slider.reset()
    omega_slider.reset()
    x0_slider.reset()
    v0_slider.reset()
button.on_clicked(reset)

plt.show()