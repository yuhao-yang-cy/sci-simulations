import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider

# Mompesson data
t_data = np.array([0, 0.51, 1.02, 1.53, 2.04, 2.55, 3.57])
I_data = np.array([15, 22, 29, 21, 8, 8, 0])
S_data = np.array([235, 201, 154, 121, 108, 97, 83])
R_data = I_data[0] + S_data[0] - I_data - S_data

def SIR_model(init_values, t, beta, gamma):
    # initialising the population size for each group
    S = np.zeros_like(t)
    I = np.zeros_like(t)
    R = np.zeros_like(t)
    
    # set up initial conditions
    S[0], I[0], R[0] = init_values
    N = sum(init_values)
    
    # use the three SIR equations to compute values of S, I, and R one time step after another
    for i in range(1, len(t)):
        if I[i-1] + (beta * S[i-1] * I[i-1] - gamma * I[i-1]) * step > N:
            I[i] = N
            S[i] = 0
        else:
            S[i] = S[i-1] - beta * S[i-1] * I[i-1] * step
            I[i] = I[i-1] + (beta * S[i-1] * I[i-1] - gamma * I[i-1]) * step
        R[i] = R[i-1] + gamma * I[i-1] * step
        
    # return outputs as three numpy arrays
    return S, I, R

# Define initial parameters
# set up simulation time
tmax = t_data[-1]
step = 0.02
t = np.arange(0, tmax+step, step)

# set up initial values for SIR model
init_values = [S_data[0], I_data[0], R_data[0]]    # S0, I0, R_0
beta = 0.01
gamma = 0.2

S, I, R = SIR_model(init_values, t, beta, gamma)
# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
line1, = ax.plot(t, S, color='blue', label='Susceptible')
line2, = ax.plot(t, I, color='red', label='Infective')
line3, = ax.plot(t, R, color='green', label='Removed')
ax.scatter(t_data, I_data, marker='x', color='red')
ax.scatter(t_data, S_data, marker='x', color='blue')
ax.scatter(t_data, R_data, marker='x', color='green')
ax.legend()
ax.set_xlabel('Time [days]')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.25, bottom=0.25)

# Make a horizontal slider to control the frequency.
axbeta = fig.add_axes([0.25, 0.1, 0.47, 0.04])
beta_slider = Slider(
    ax=axbeta,
    label=r'$\beta$',
    valmin=0,
    valmax=0.05,
    valinit=beta,
)

axgamma = fig.add_axes([0.25, 0.05, 0.47, 0.04])
gamma_slider = Slider(
    ax=axgamma,
    label=r'$\gamma$',
    valmin=0,
    valmax=5,
    valinit=gamma,
)

# The function to be called anytime a slider's value changes
def update(val):
    S, I, R = SIR_model(init_values, t, beta_slider.val, gamma_slider.val)
    line1.set_ydata(S)
    line2.set_ydata(I)
    line3.set_ydata(R)
    fig.canvas.draw_idle()

# register the update function with each slider
beta_slider.on_changed(update)
gamma_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.81, 0.07, 0.08, 0.05])
button = Button(resetax, 'Reset', hovercolor='0.975')


def reset(event):
    beta_slider.reset()
    gamma_slider.reset()
button.on_clicked(reset)

plt.show()