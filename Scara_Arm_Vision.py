import cv2 as cv
import numpy as np
import serial

maxValueOfPixel = 255
escapeKey = 27
threshold = 100
pixelToCm = 49/640.0


def send_coordinates(x, y):
    coordinates = f"{x},{y}\n"
    serialPort.write(coordinates.encode('utf-8'))


Rotation_180_X = [[1, 0, 0], [0, np.cos(np.pi), -np.sin(np.pi)], [0, np.sin(np.pi), np.cos(np.pi)]]
Angle_90_in_Rad = (-183.0/180.0) * np.pi  # add the additional degrees to the 90deg
Rotation_90_Z = [[np.cos(Angle_90_in_Rad), -np.sin(Angle_90_in_Rad), 0],
                 [np.sin(Angle_90_in_Rad), np.cos(Angle_90_in_Rad), 0],
                 [0, 0, 1]
                 ]
CameraRotationMatrix = np.dot(Rotation_180_X, Rotation_90_Z)
''' if we place the arm at the position 0,0,0 of the camera we do not need a displacement vector
but if we don't, then
displace_of_origin_to_camera = [[x], [y], 0]
'''
displace_of_origin_to_camera = [[22], [0.8], [0]]
HomogeneousMatrixFromCamToOrigin = np.concatenate((CameraRotationMatrix, displace_of_origin_to_camera), 1)
HomogeneousMatrixFromCamToOrigin = np.concatenate((HomogeneousMatrixFromCamToOrigin, [[0, 0, 0, 1]]), 0)

# Camera Instantiation
cap = cv.VideoCapture(0)

while True:
    _, frame = cap.read()
    background = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow("background", background)
    cv.imshow("b", frame)

    esc = cv.waitKey(1)
    if esc == escapeKey:
        break

while True:
    _, frame = cap.read()
    foreground = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow("foreground", foreground)
    diff = np.absolute(np.matrix(np.int16(background)) - np.matrix(np.int16(foreground)))
    diff[diff > maxValueOfPixel] = maxValueOfPixel
    diff = np.uint8(diff)
    cv.imshow("diff", diff)

    blackAndWhite = diff
    blackAndWhite[blackAndWhite <= threshold] = 0
    blackAndWhite[blackAndWhite > threshold] = 1

    xCord = np.matrix(np.sum(blackAndWhite, 0))
    xCordNumber = np.matrix(np.arange(640))
    xCordMultiply = np.multiply(xCord, xCordNumber)
    xTotal = np.sum(xCordMultiply)
    xTotal_total = np.sum(np.sum(blackAndWhite))
    xLocationInPixel = xTotal / xTotal_total
    xLocationInCm = xLocationInPixel * pixelToCm

    yCordSum = np.matrix(np.sum(blackAndWhite, 1))
    yCord = yCordSum.transpose()
    yCordNumber = np.matrix(np.arange(480))
    yCordMultiply = np.multiply(yCord, yCordNumber)
    yTotal = np.sum(yCordMultiply)
    yTotal_total = np.sum(np.sum(blackAndWhite))
    yLocationInPixel = yTotal / yTotal_total
    yLocationInCm = yLocationInPixel * pixelToCm

    PointInCam = [[xLocationInCm], [yLocationInCm], [0], [1]]
    CordInBase = np.dot(HomogeneousMatrixFromCamToOrigin, PointInCam)

    xCordInBase = CordInBase[0]
    yCordInBase = CordInBase[1]

    print("the X coordinate is: %f and the Y coordinate is: %f" % (xCordInBase, yCordInBase))

    esc = cv.waitKey(1)
    if esc == escapeKey:
        break

cv.destroyAllWindows()

a = float(xCordInBase)
b = float(yCordInBase)
serialPort = serial.Serial('COM8', 9600)

serialPort.write(f"{a} {b}\n".encode())

serialPort.close()
