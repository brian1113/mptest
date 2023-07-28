import trapezoid as trap
import matplotlib.pyplot as plt
import numpy as np

a = trap.MotionState(10, 5, 0)
b = trap.MotionState(30, 0, 0)
p = trap.Trapezoid(a, b)

dt = 0.01
duration = 3
time = np.linspace(0, duration, int(duration / dt))

calculated_distances = [p.calculate_distance(10, 8, 2, t, False) for t in time]
calculated_velocities = [p.calculate_distance(10, 8, 2, t, True) for t in time]


plt.plot(time, calculated_distances)
plt.plot(time, calculated_velocities)
plt.show()
