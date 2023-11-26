import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider

def reduced_model(init_values, t, r):
    # SIR model in terms of three differential equations
    S = np.zeros_like(t)
    I = np.zeros_like(t)
    R = np.zeros_like(t)
    
    S[0], I[0], R[0] = init_values
    
    for i in range(1, len(t)):
        S[i] = S[i-1] - r * S[i-1] * I[i-1] * step
        I[i] = I[i-1] + (r * S[i-1] * I[i-1] - I[i-1]) * step
        R[i] = R[i-1] + I[i-1] * step
    return S, I, R

# Define initial parameters
# set up simulation time
tmax = 30
step = 0.01
t = np.arange(0, tmax, step)

# set up initial values for SIR model
init_values = [0.995, 0.005, 0]     # S0, I0, R_0
r = 2

S, I, R = reduced_model(init_values, t, r)
# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
line1, = ax.plot(t, S, label='Susceptible')
line2, = ax.plot(t, I, label='Infective')
line3, = ax.plot(t, R, label='Removed')
ax.legend()
ax.set_xlabel('Time [days]')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.25, bottom=0.25)

# Make a horizontal slider to control the frequency.
axr = fig.add_axes([0.25, 0.1, 0.48, 0.04])
r_slider = Slider(
    ax=axr,
    label=r'$\mathcal{R}_0$',
    valmin=0,
    valmax=12,
    valinit=r,
)

# The function to be called anytime a slider's value changes
def update(val):
    S, I, R = reduced_model(init_values, t, r_slider.val)
    line1.set_ydata(S)
    line2.set_ydata(I)
    line3.set_ydata(R)
    fig.canvas.draw_idle()

# register the update function with each slider
r_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.81, 0.1, 0.08, 0.05])
button = Button(resetax, 'Reset', hovercolor='0.975')


def reset(event):
    r_slider.reset()
button.on_clicked(reset)

plt.show()