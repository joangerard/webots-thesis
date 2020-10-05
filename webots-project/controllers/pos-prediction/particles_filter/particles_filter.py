import numpy as np


class ParticlesFilter:
    def __init__(self, environment_config, robot_initial_config, predictor, params):
        """
        Initialize the particles filter parameters.

        :param environment_config: EnvironmentConfiguration
        :param robot_initial_config:  RobotConfiguration
        :param predictor: PredictorNNSensorsNotNormalized
        :param params: Params
        """
        # params received from --cli execution
        self.params = params
        # media
        self.mu = 0
        # standard deviation for translation
        self.sigma_xy = self.params.SIGMA_XY
        # standard deviation for rotation
        self.sigma_theta = self.params.SIGMA_THETA
        # environment configuration parameters
        self.environment_config = environment_config
        # robot initial state
        self.robot_previous_config = robot_initial_config
        # number of particles: 300,500,1000, etc...
        self.number_of_particles = self.params.PARTICLES_NUMBER
        # this object contains the trained models
        self.predictor = predictor

        # weights or importance factors for M particles
        self.weights = [1/self.number_of_particles] * self.number_of_particles

        # initial state for all the particles
        if not self.params.GLOBAL_LOCALIZATION:
            x = np.random.normal(robot_initial_config.x, self.sigma_xy, self.number_of_particles)
            y = np.random.normal(robot_initial_config.y, self.sigma_xy, self.number_of_particles)
            theta = np.random.normal(robot_initial_config.theta, self.sigma_theta, self.number_of_particles)
        else:
            x = np.random.uniform(0, environment_config.environment_dim_x, self.number_of_particles)
            y = np.random.uniform(0, environment_config.environment_dim_y, self.number_of_particles)
            theta = np.random.uniform(0, 360, self.number_of_particles)

        # define the set of particles as [x, y, theta, weights]
        self.particles = np.array([x, y, theta, self.weights])

    def reset_particles(self, number_particles, sigma_xy, sigma_theta, position):
        """
        This function is useful for changing the algorithm parameters while running the simulation.
        It resets the number of particles and the noise.

        :param number_particles: int
        :param sigma_xy: float
        :param sigma_theta: float
        :param position: RobotConfiguration

        :return: new brand new set of particles [x, y, theta, weights]
        """

        # reset parameters
        self.number_of_particles = int(number_particles)
        self.sigma_xy = float(sigma_xy)
        self.sigma_theta = float(sigma_theta)

        # reset weight
        self.weights = [1/self.number_of_particles for i in range(self.number_of_particles)]

        # reset particles state
        x = np.random.normal(position.x, self.sigma_xy, self.number_of_particles)
        y = np.random.normal(position.y, self.sigma_xy, self.number_of_particles)
        theta = np.random.normal(position.theta, self.sigma_theta, self.number_of_particles)

        self.particles = np.array([x, y, theta, self.weights])

    def get_particles(self, delta_movement, sensors, resampling=True):
        """
        This is the CORE of the particles filter algorithm.
        It generates a new set of particles
        given the control action delta_movement, the sensors
        measurements and it applies resampling
        in the case that resampling parameter is set to true.

        :param delta_movement: Array containing the
                                control action [d_x, d_y, d_theta]
        :param sensors: Array containing sensor measurements
        :param resampling: Boolean if true it applies resampling
        :return: [x, y, theta, self.weights] set of particles
        """
        # move the particles
        self._apply_movement_to(self.particles, delta_movement)

        particle_transpose = self.particles[0:3].transpose()

        # calculate weights based on the models and the sensor measurements
        self.particles[3], bad_data = self.predictor.get_particles_weight(particle_transpose,
                                                                          sensors)

        # normalize weights
        self.normalize_weights()

        # resampling based on weights
        if resampling:
            indexes = range(0, self.number_of_particles)
            indexes = np.random.choice(indexes,
                                       self.number_of_particles,
                                       p=self.particles[3],
                                       replace=True)
            self.particles[0:3] = particle_transpose[indexes].transpose()

        return self.particles

    def normalize_weights(self):
        """
        Normalize weights to sum up to 1
        """
        self.particles[3] = self.particles[3] / self.particles[3].min()
        self.particles[3] = self.particles[3] / self.particles[3].sum()

    def _apply_movement_to(self, particles, delta_robot_config):
        """
        Apply the control action delta_robot_config to all the particles + some noise.

        :param particles: [x, y, theta, weight]
        :param delta_robot_config: [x, y, theta]
        """
        # get noise
        noise_x = np.random.normal(self.mu, self.sigma_xy, self.number_of_particles)
        noise_y = np.random.normal(self.mu, self.sigma_xy, self.number_of_particles)
        noise_theta = np.random.normal(self.mu, self.sigma_theta, self.number_of_particles)

        # move particles
        particles[0] += delta_robot_config[0] + noise_x
        particles[1] += delta_robot_config[1] + noise_y
        particles[2] += delta_robot_config[2] + noise_theta

        # clip the positioning to not be outside the arena or grater than 360 degrees on orientation
        particles[0] = np.clip(particles[0], 0, self.environment_config.environment_dim_x)
        particles[1] = np.clip(particles[1], 0, self.environment_config.environment_dim_y)
        particles[2] = (360 + particles[2]) % 360
