import numpy as np
import matplotlib.pyplot as plt
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage

K_P = 3.0
K_I = 1.0
K_D = 0.1
 
STEPS = 550
USE_HIGH_FRICTION = True
 
car = make_car(desired_v=20.0, dt=0.1)

velocities = []
errors = []
times = []

for step in range(STEPS):
    acceleration_desired, error = calculate_desired_acceleration(car, K_P, K_I, K_D)
    throttle_percentage = acceleration_to_throttle_percentage(acceleration_desired)

    if USE_HIGH_FRICTION and 150 <= step < 300:
        friction = 4.0
    else:
        friction = 2.0

    update(car, throttle_percentage, friction=friction)

    velocities.append(car["v"])
    errors.append(error)
    times.append(car["t"])

print("final velocity:", velocities[-1])
print("final error:", errors[-1])

fig, (ax_velocity, ax_error) = plt.subplots(2, 1, sharex=True) 
# create a figure with 2 subplots, sharing the x-axis

ax_velocity.plot(times, velocities)
ax_velocity.set_ylabel("Velocity")
ax_velocity.axhline(car["desired_v"], color="gray", linestyle="--")
ax_velocity.set_ylim(0, car["desired_v"] + 2)
if USE_HIGH_FRICTION:
    ax_velocity.axvspan(15, 30, color="red", alpha=0.1, label="High friction")
ax_velocity.set_title(f"PID: K_P={K_P}, K_I={K_I}, K_D={K_D}")
ax_velocity.legend()

ax_error.plot(times, errors)
ax_error.axhline(0, color="gray", linestyle="--") 
ax_error.set_ylabel("Error")
ax_error.set_xlabel("Time")

plt.show()
