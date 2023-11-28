import kalman
import sim
import csv
import random
import numpy as np

def parseData():
    data = []

    with open('test.csv', 'r') as csv_file:
        next(csv_file)
        reader = csv.reader(csv_file)


        for row in reader:
            temp1 = []
            for element in row:
                temp1.append(float(element))

            data.append(temp1.copy())

    return data

def generateMeasurements(data):
    # We can use gaussian distributions for this
    # The mean will be the actual value from the simulation and the standard deviation will be 3 m for position
    # and .05 m/s for velocity

    # In format [x, vx, y, vy, z, vz]
    measurement = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

    for i in range(0, len(data), 2):
        measurement[i][0] = random.gauss(data[i], 3)

    for i in range(1, len(data), 2):
        measurement[i][0] = random.gauss(data[i], .05)

    return measurement

def main():
    simulator = sim.Sim()
    kalFilter = kalman.Kalman_Filter()

    simulator.simulate()
    data = parseData()

    initialConditions = data[0][1::]
    kalFilter.updateInitialConditions(initialConditions)

    for i in range(1, len(data)):
        # Predict
        kalFilter.predict()

        # Measurement Update
        measurement = generateMeasurements(data[i][1::])
        kalFilter.update(measurement)


if __name__ == '__main__':
    main()