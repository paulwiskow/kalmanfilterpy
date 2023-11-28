import kalman
import sim
import csv
import random
import numpy as np
import matplotlib.pyplot as plt

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


# Returns in form [[rawdata x], [rawdata vx], [rawdata y], [rawdata vy], [rawdata z], [rawdata vz], [kalmanData x], [kalmanData vx], [kalmanData y], [kalmanData vy], [kalmanData z], [kalmanData vz]]
def generatePlottingData(rawData, kalmanData):
    rawX = []
    rawVx = []
    rawY = []
    rawVy = []
    rawZ = []
    rawVz = []

    kalX = []
    kalVx = []
    kalY = []
    kalVy = []
    kalZ = []
    kalVz = []

    for data in rawData:
        # time is the first element in the raw data
        rawX.append(data[1])
        rawVx.append(data[2])
        rawY.append(data[3])
        rawVy.append(data[4])
        rawZ.append(data[5])
        rawVz.append(data[6])

    for data in kalmanData:
        # weird numpy formatting when I transposed this initially
        kalX.append(data[0][0])
        kalVx.append(data[0][1])
        kalY.append(data[0][2])
        kalVy.append(data[0][3])
        kalZ.append(data[0][4])
        kalVz.append(data[0][5])

    return [rawX, rawVx, rawY, rawVy, rawZ, rawVz, kalX, kalVx, kalY, kalVy, kalZ, kalVz]


def main():
    simulator = sim.Sim()
    kalFilter = kalman.Kalman_Filter()

    simulator.simulate()
    data = parseData()

    initialConditions = generateMeasurements(data[0][1::])
    kalFilter.updateInitialConditions(initialConditions)

    kalmanData = [np.transpose(initialConditions)]

    for i in range(1, len(data)):
        # Predict
        kalFilter.predict()

        # Measurement Update
        measurement = generateMeasurements(data[i][1::])
        kalFilter.update(measurement)

        kalmanData.append(np.transpose(kalFilter.currentStateVector))

    # setup time (x axis)
    time = 0
    timeAxis = []
    for i in range(len(data)):
        timeAxis.append(time)
        time += .1

    allData = generatePlottingData(data, kalmanData)

    # X coordinate plotting
    plt.scatter(timeAxis, allData[0], label="Raw data")
    plt.scatter(timeAxis, allData[6], label="Kalman Filter")
    plt.title("X Coordinate")
    plt.legend()
    plt.show()

    # VX coordinate plotting
    plt.scatter(timeAxis, allData[1], label="Raw data")
    plt.scatter(timeAxis, allData[7], label="Kalman Filter")
    plt.title("VX")
    plt.legend()
    plt.show()

    # Y coordinate plotting
    plt.scatter(timeAxis, allData[2], label="Raw data")
    plt.scatter(timeAxis, allData[8], label="Kalman Filter")
    plt.title("Y Coordinate")
    plt.legend()
    plt.show()

    # VY coordinate plotting
    plt.scatter(timeAxis, allData[3], label="Raw data")
    plt.scatter(timeAxis, allData[9], label="Kalman Filter")
    plt.title("VY")
    plt.legend()
    plt.show()

    # Z coordinate plotting
    plt.scatter(timeAxis, allData[4], label="Raw data")
    plt.scatter(timeAxis, allData[10], label="Kalman Filter")
    plt.title("Z Coordinate")
    plt.legend()
    plt.show()

    # VZ coordinate plotting
    plt.scatter(timeAxis, allData[5], label="Raw data")
    plt.scatter(timeAxis, allData[11], label="Kalman Filter")
    plt.title("VZ")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
