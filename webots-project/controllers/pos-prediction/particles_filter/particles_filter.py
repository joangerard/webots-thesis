import numpy as np


class ParticlesFilter:
    def __init__(self, environment_config, robot_initial_config, predictor, params):
        self.params = params
        self.margin_replacement = 0.5
        self.mu = 0
        self.sigma_xy = self.params.SIGMA_XY
        self.sigma_theta = self.params.SIGMA_THETA
        self.environment_config = environment_config
        self.robot_previous_config = robot_initial_config
        self.number_of_particles = self.params.PARTICLES_NUMBER
        self.predictor = predictor
        self.alpha = 0.7
        self.weights = [1/self.number_of_particles for i in range(self.number_of_particles)]
        x = np.random.normal(robot_initial_config.x, self.sigma_xy, self.number_of_particles)
        y = np.random.normal(robot_initial_config.y, self.sigma_xy, self.number_of_particles)
        theta = np.random.normal(robot_initial_config.theta, self.sigma_theta, self.number_of_particles)
        # x = np.random.uniform(0, environment_config.environment_dim_x, self.number_of_particles)
        # y = np.random.uniform(0, environment_config.environment_dim_y, self.number_of_particles)
        # theta = np.random.uniform(0, 360, self.number_of_particles)
        self.particles = np.array([x, y, theta, self.weights])

    def get_particles(self, delta_movement, sensors, resampling=True):

        self._apply_movement_to(self.particles, delta_movement)

        particle_transpose = self.particles[0:3].transpose()

        # calculate weights
        self.particles[3], bad_data = self.predictor.prediction_error(particle_transpose, sensors)

        self.normalize_weights()

        # resampling based on weights
        if resampling:
            indexes = np.random.choice(range(0, self.number_of_particles), self.number_of_particles, p=self.particles[3], replace=True)
            self.particles[0:3] = particle_transpose[indexes].transpose()

        return self.particles

    def normalize_weights(self):
        self.particles[3] = self.particles[3] / self.particles[3].min()
        self.particles[3] = self.particles[3] / self.particles[3].sum()

    def _apply_movement_to(self, particles, delta_robot_config):
        std_x = np.random.normal(self.mu, self.sigma_xy, self.number_of_particles)
        std_y = np.random.normal(self.mu, self.sigma_xy, self.number_of_particles)
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
