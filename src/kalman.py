import numpy as np

class Kalman_Filter:
    def __init__(self):
        self.deltaTime = .1
        self.identityMatrix = np.diag([1.0, 1, 1, 1, 1, 1])
        self.kalmanGain = np.array([[0.0, 0, 0, 0, 0, 0], # redefined later, but just defined here for now
                                [0, 0.0, 0, 0, 0, 0],
                                [0, 0, 0.0, 0, 0, 0],
                                [0, 0, 0, 0.0, 0, 0],
                                [0, 0, 0, 0, 0.0, 0],
                                [0, 0, 0, 0, 0, 0.0]])

        self.previousStateVector = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]) # used to predict acceleration
        self.currentStateVector = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
        self.predictedStateVector = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
        self.acceleration = np.array([[0.0], [0.0], [0.0]]) # need to derive this separately by taking derivative of velocity change

        self.stateTransition = np.array([[1, 0, 0, self.deltaTime, 0, 0], # basically assuming newtonian motion here
                                [0, 1, 0, 0, self.deltaTime, 0],
                                [0, 0, 1, 0, 0, self.deltaTime],
                                [0, 0, 0, 1.0, 0, 0],
                                [0, 0, 0, 0, 1.0, 0],
                                [0, 0, 0, 0, 0, 1.0]])
        self.control = np.array([[.5 * self.deltaTime**2, 0, 0], # taking into account our derived acceleration
                        [0, .5 * self.deltaTime**2, 0],
                        [0, 0, 0.5 * self.deltaTime**2],
                        [self.deltaTime, 0, 0],
                        [0, self.deltaTime, 0],
                        [0, 0, self.deltaTime]])
        self.observationMatrix = np.array([[1.0, 0, 0, 0, 0, 0],
                                [0, 1.0, 0, 0, 0, 0],
                                [0, 0, 1.0, 0, 0, 0],
                                [0, 0, 0, 1.0, 0, 0],
                                [0, 0, 0, 0, 1.0, 0],
                                [0, 0, 0, 0, 0, 1.0]])

        # These are the most sus out of all the things, really unsure about how accurate these matrixes are
        self.processNoiseMatrix = np.diag([1.0, 1, 1, 1, 1, 1]) # noise in each of the gps measurements from immeasurable factors, fine tune this with simulation
        self.measurementNoiseMatrix = np.diag([3.0, .005, 3.0, .005, 3.0, .005]) # this will most likely depend on how many satellites we have locked, but we can initialize with constants here

        self.currentCovariance = np.diag([20.0, 10, 20, 10, 20, 10])
        self.predictedCovariance = np.diag([0.0, 0, 0, 0, 0, 0])

    # initialConditions in form [x, vx, y, vy, z, vz]
    def updateInitialConditions(self, initialConditions):
        for i in range(len(initialConditions)):
            self.currentStateVector[i][0] = initialConditions[i]
            self.previousStateVector[i][0] = initialConditions[i]

    def predict(self):
        self.__updateAcceleration()
        self.__predictState()
        self.__predictCovariance()

    def __predictState(self):
        self.previousStateVector = self.currentStateVector.copy()
        self.predictedStateVector = np.add(np.dot(self.stateTransition, self.currentStateVector), np.dot(self.control, self.acceleration))

    def __predictCovariance(self):
        self.predictedCovariance = np.add(np.dot(np.dot(self.stateTransition, self.currentCovariance), np.transpose(self.stateTransition)), self.processNoiseMatrix)

    def __updateAcceleration(self):
        j = 0
        for i in range(1, len(self.previousStateVector), 2):
            self.acceleration[j][0] = (self.currentStateVector[i][0] - self.previousStateVector[i][0]) / self.deltaTime
            j += 1

    def update(self, measurement):
        self.__updateKalmanGain()
        self.__updateState(measurement)
        self.__updateCovariance()

    def __updateKalmanGain(self):
        temp1 = np.dot(self.predictedCovariance, np.transpose(self.observationMatrix))
        temp2 = np.add(np.dot(np.dot(self.observationMatrix, self.predictedCovariance), np.transpose(self.observationMatrix)), self.measurementNoiseMatrix)
        temp3 = np.linalg.inv(temp2)

        self.kalmanGain = np.dot(temp1, temp3)

    def __updateState(self, measurement):
        self.currentStateVector = np.add(self.predictedStateVector, np.dot(self.kalmanGain, np.subtract(measurement, np.dot(self.observationMatrix, self.predictedStateVector))))

    def __updateCovariance(self):
        temp1 = np.subtract(self.identityMatrix, np.dot(self.kalmanGain, self.observationMatrix))
        temp2 = np.transpose(temp1)
        temp3 = np.dot(np.dot(self.kalmanGain, self.measurementNoiseMatrix), np.transpose(self.kalmanGain))

        self.currentCovariance = np.add(np.dot(np.dot(temp1, self.predictedCovariance), temp2), temp3)

