from sim.a35 import plot

import random as r
import numpy as np


class Robot:
    def __init__(self, pos):
        self.pos = pos
        self.move_dist = 1


class Particle(Robot):
    def __init__(self, pos):
        Robot.__init__(self, pos)
        self.weight = 0
        self.movement_sigma = 3
        self.move_dist = 4  # Overwrite move distance for this example
        self.color = (0, 0, 1, 1)

    def predict(self):
        ### START STUDENT CODE
        self.pos = np.random.normal(self.pos + self.move_dist,self.movement_sigma)
        ### END STUDENT CODE

def resample_particles(particles):
    ### START STUDENT CODE
    # Please fill this array with the output of the r.choices function.
    weights = []
    for particle in particles:
        weights += [particle.weight]
    resample = r.choices(range(num_particles),weights,k=num_particles)  # range(num_particles) = [0,1,2,3,4,5,6,7,8,9]
    print("weights: "+ str(weights))  # ilk üç particleınki 1 diğerlerinin weight'i 0.1
    print("resample: "+ str(resample)) # weights oranlarına göre ortaya çıkan yeni particle lar. (0,1,2) nolu particllerın olma olasılığı 1 iken diğerleri 0.1
    
    
    # Please fill this array with resampled partciles.
    resampled_particles = []
    for i in resample:
        resampled_particles += [Particle(particles[i].pos)]
    ### END STUDENT CODE

    # Set all resampled particles to a different color.
    for i in resample:
        resampled_particles[-1].color = (0, 1, 0, 1)

    # Plot and return resampled particles.
    plot(particles, resampled_particles, resample, distance)
    return resampled_particles


num_particles = 10
distance = 40
particles = []

# Set first few particles to different weight and color.
for i in range(num_particles):
    particles += [Particle(r.uniform(0, distance))]
    if i < 3:
        particles[-1].weight = 1
        particles[-1].color = (1, 0, 0, 1) # red ones
    else:
        particles[-1].weight = 0.1 #blue ones

# Resample Particles
resampled_particles = resample_particles(particles)
