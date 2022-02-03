import matplotlib.pyplot as plt
import numpy as np
import random as r
import math
from sim.plot import plot, print_particle_error


AUTORUN = False
robot_start = 7
num_particles = 20
distance = 40
poles = [10, 15, 17, 19, 30, 39]


### START STUDENT CODE
class Robot:
    def __init__(self, pos):
        self.pos = pos
        self.move_dist = 1
        self.max_measurement = 3
        self.pole_dist = -100
    
    # Movement is perfectly accurate, even though we are assuming it isn't.
    def move(self):
        self.pos += self.move_dist
    
    # Measurement is perfectly accurate even though we are assuming it isn't.
    def measure(self, poles):
        potential_detected_poles = []
        for pole in poles:
            difference = pole - self.pos
            if difference > 0:
                potential_detected_poles += [difference]
        if len(potential_detected_poles) == 0:
            self.pole_dist = -100
        else:
            self.pole_dist = min(potential_detected_poles)
            if self.pole_dist > self.max_measurement:
                self.pole_dist = -100
        #print(difference)

class Particle(Robot):
    def __init__(self, pos):
        Robot.__init__(self, pos)
        self.weight = 0
        self.movement_sigma = 0.2
        self.measurement_sigma = 1.0
    
    def predict(self):
        self.pos = np.random.normal(self.pos + self.move_dist,self.movement_sigma)

    def probability_density_function(self, mu, x):
        pdf = (np.exp((((x - mu)/self.measurement_sigma)**2)*(-0.5)))/(self.measurement_sigma*np.sqrt(2*np.pi))
        return pdf
    def update_weight(self, robot_dist):
        self.weight = particle.probability_density_function(robot_dist, self.pole_dist)

def resample_particles(particles):
    weights = []
    for particle in particles:
        weights += [particle.weight]
    # Potentially resample uniformly if weights are so low.
    if sum(weights)< 0.05:
        resampled_particles = []
        for i in range(len(particles)):
            resampled_particles += [Particle(r.uniform(0,39))]
        return resampled_particles
    
    resample = r.choices(range(num_particles),weights,k=num_particles)  # range(num_particles) = [0,1,2,3,4,5,6,7,8,9]
    resampled_particles = []
    for i in resample:
        resampled_particles += [Particle(particles[i].pos)]
    return resampled_particles

def initialize_particles(particles):
    #for i in range(num_particles):
        #particles += [Particle(r.uniform(0, distance-1))]
    # not randomly, perfect uniform distribution. belli boşluklarla parrticle ları koyduk.
    diff = distance / num_particles
    for i in range(num_particles):
        particles += [Particle(diff*i)]

robot = Robot(robot_start)

# Setup particles.
particles = []
initialize_particles(particles)

# Plot starting distribution, no beliefs
plot(particles, poles, robot.pos)

# Begin Calculating
for j in range(39 - robot.pos):
    # Move
    if j != 0:
        robot.move()
        for particle in particles:
            particle.predict()

    # Measure
    robot.measure(poles)
    for particle in particles:
        particle.measure(poles)

        # Update Beliefs
        particle.update_weight(robot.pole_dist)

    print_particle_error(robot, particles) # error : weight i en yüksek olan particle ile robot arasındaki mesafe

    # Resample
    resampled_particles = resample_particles(particles)
    plot(particles, poles, robot.pos, resampled_particles, j, AUTORUN)
    particles = resampled_particles

plot(particles, poles, robot.pos, resampled_particles)
