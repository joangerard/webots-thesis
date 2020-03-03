import random
import numpy as np
import math
from robot_configuration import RobotConfiguration
from weighted_population import WeightedPopulation


class ParticlesFilter:
    def __init__(self, environment_config, robot_initial_config, predictor):
        self.margin_replacement = 0.5
        self.mu = 0
        self.sigma = 0.00045
        self.sigma_theta = 2.5
        self.environment_config = environment_config
        self.robot_previous_config = robot_initial_config
        self.number_of_particles = 500
        self.predictor = predictor
        self.weights = [1/self.number_of_particles for i in range(self.number_of_particles)]
        x = np.random.normal(robot_initial_config.x, self.sigma, self.number_of_particles)
        y = np.random.normal(robot_initial_config.y, self.sigma, self.number_of_particles)
        theta = np.random.normal(robot_initial_config.theta, self.sigma_theta, self.number_of_particles)
        # x = np.random.uniform(0, environment_config.environment_dim_x, self.number_of_particles)
        # y = np.random.uniform(0, environment_config.environment_dim_y, self.number_of_particles)
        # theta = np.random.uniform(0, 360, self.number_of_particles)
        self.particles = np.array([x, y, theta, self.weights])

    def get_particles(self, delta_movement, sensors, apply_movement=True):

        self._apply_movement_to(self.particles, delta_movement)

        # calculate weights
        for particle in self.particles.transpose():
            particle[3], bad_data = self.predictor.prediction_error(particle[0], particle[1], particle[2], sensors)

        # normalize weights
        self.normalize_weights()

        # resampling based on weights
        indexes = np.random.choice(range(0, self.number_of_particles), self.number_of_particles, p=self.particles[3], replace=True)
        self.particles = self.particles.transpose()[indexes].transpose()

        return self.particles

    def normalize_weights(self):
        self.particles[3] = self.particles[3] / self.particles[3].min()
        self.particles[3] = self.particles[3] / self.particles[3].sum()

    def sampling_from_best(self):
        number_new_particles = math.floor(self.number_of_particles/500)
        particles = self.particles.transpose()
        particles = sorted(particles, key=lambda particle: particle[3], reverse=True)

        # replace 1/5 of population
        best_x = particles[0][0]
        best_y = particles[0][1]
        best_weight = particles[0][3]

        new_best_xs = np.random.uniform(best_x - self.margin_replacement,
                                        best_x + self.margin_replacement,
                                        number_new_particles)
        new_best_ys = np.random.uniform(best_y - self.margin_replacement,
                                        best_y + self.margin_replacement,
                                        number_new_particles)
        new_best_thetas = np.random.uniform(0,
                                            360,
                                            number_new_particles)

        new_best_particles = np.array([new_best_xs, new_best_ys, new_best_thetas, [best_weight for i in range(0, number_new_particles)]])

        final_particles = np.vstack((particles[:len(particles) - number_new_particles], new_best_particles.transpose()))

        return final_particles.transpose()


    def _weighted_sample(self, population, weights, k):
        indexes = random.sample(WeightedPopulation(range(0, self.number_of_particles), weights), k)
        return population[indexes]

    def _calculate_delta_movement(self, previous_robot_config, actual_robot_config):
        x = actual_robot_config.x - previous_robot_config.x
        y = actual_robot_config.y - previous_robot_config.y
        theta = actual_robot_config.theta - previous_robot_config.theta

        return RobotConfiguration(x, y, theta)

    def _apply_movement_to(self, particles, delta_robot_config):
        std_x = np.random.normal(self.mu, self.sigma, self.number_of_particles)
        std_y = np.random.normal(self.mu, self.sigma, self.number_of_particles)
        std_theta = np.random.normal(self.mu, self.sigma_theta, self.number_of_particles)
        # std_x = delta_robot_config.x
        # std_y = delta_robot_config.y
        # std_theta = delta_robot_config.theta

        particles[0] += delta_robot_config[0] + std_x
        particles[1] += delta_robot_config[1] + std_y
        particles[2] += delta_robot_config[2] + std_theta

        particles[0] = np.clip(particles[0], 0, self.environment_config.environment_dim_x)
        particles[1] = np.clip(particles[1], 0, self.environment_config.environment_dim_y)
        particles[2] = (360 + particles[2]) % 360
