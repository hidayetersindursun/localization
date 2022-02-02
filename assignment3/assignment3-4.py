import matplotlib.pyplot as plt
import math
import numpy as np
import random as r


class Robot:
    def __init__(self, pos):
        self.pos = pos
        self.pole_dist = 0


class Particle(Robot):
    def __init__(self, pos):
        Robot.__init__(self, pos)
        self.weight = 0
        self.measurement_sigma = 1

    def probability_density_function(self, mu, x):
        ### STUDENT CODE START
        pdf = (np.exp((((x - mu)/self.measurement_sigma)**2)*(-0.5)))/(self.measurement_sigma*np.sqrt(2*np.pi))  
        #return 1.0 gibi bi şey
        return pdf
        ### STUDENT CODE END

    def update_weight(self, robot_dist):
        ### STUDENT CODE START
        
        self.weight = particle.probability_density_function(robot_dist, self.pole_dist)
        # self.weight i güncelle 
        
        ### STUDENT CODE START
        

# Plot Weights for a range of robot measurements.
particle = Particle(0.0)
x = np.arange(-5, 5, 0.01)
y = np.zeros(len(x))
for i in range(len(x)):
    particle.update_weight(x[i])
    y[i] = particle.probability_density_function(0, x[i])

plt.plot(x, y, '-r')
plt.grid(True)
plt.show()

# Integrate left side to calculate probablity.
sum_probability = 0
for i in range(int(len(y) / 2)):
    sum_probability += y[i]

print("If Probability is close to 0.5, then PDF works.")
print(round(sum_probability * 0.01, 2))
print()

# Update Particle Weigth based on robot measurement.
robot_dist = 4.0
particle.pole_dist = 3.0
particle.update_weight(robot_dist)
print("Particle Weight: " + str(round(particle.weight, 2)))
plt.plot(x, y, '-r')
plt.plot([-5, 5], [particle.weight, particle.weight], '-b')
plt.grid(True)
plt.show()
