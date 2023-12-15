import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

# Example data (replace this with your real telemetry data)
speed_data = np.random.rand(10)
steering_data = np.random.rand(10)

# Set up the Tkinter window
window = tk.Tk()
window.title("Vehicle Telemetry")

# Create a Matplotlib figure and axes
fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.plot(speed_data, label='Speed')
ax2.plot(steering_data, label='Steering Angle')

# Add a canvas to the Tkinter window
canvas = FigureCanvasTkAgg(fig, master=window)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack()

# Update the plots with new data (this part should be in your data update loop)
def update_plots(new_speed_data, new_steering_data):
    ax1.clear()
    ax2.clear()
    ax1.plot(new_speed_data, label='Speed')
    ax2.plot(new_steering_data, label='Steering Angle')
    canvas.draw()

# Example update (replace this with real-time data update)
update_plots(np.random.rand(10), np.random.rand(10))

window.mainloop()