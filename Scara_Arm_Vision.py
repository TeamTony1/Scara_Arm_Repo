# importing modules
import cv2 as cv
import numpy as np
import serial

# variable declaration
maxValueOfPixel = 255
escapeKey = 27
threshold = 100
pixelToCm = 49/640.0


# function to send coordinates
def send_coordinates(x, y): 
    coordinates = f"{x},{y}\n"
    serialPort.write(coordinates.encode('utf-8'))


Rotation_180_X = [[1, 0, 0], [0, np.cos(np.pi), -np.sin(np.pi)], [0, np.sin(np.pi), np.cos(np.pi)]] # 180deg rotation along the X-axis
Angle_90_in_Rad = (-90.0/180.0) * np.pi  # convert 90deg to radian
Rotation_90_Z = [[np.cos(Angle_90_in_Rad), -np.sin(Angle_90_in_Rad), 0],
                 [np.sin(Angle_90_in_Rad), np.cos(Angle_90_in_Rad), 0],
                 [0, 0, 1]
                 ] # 90deg rotational matrix alone the Z-axis
CameraRotationMatrix = np.dot(Rotation_180_X, Rotation_90_Z) # camera rotation matrix for the camera frame to the base frame of the arm

displace_of_origin_to_camera = [[22], [0.8], [0]] # displacement vector of the camera to the origin of the arm

# concatenate the camera  rotation matrix to the displacement vector 
HomogeneousMatrixFromCamToOrigin = np.concatenate((CameraRotationMatrix, displace_of_origin_to_camera), 1) 

# concatenate the camera  rotational translational to the [0,0,0,1] to get the Homogeneous Transformation Matrix
HomogeneousMatrixFromCamToOrigin = np.concatenate((HomogeneousMatrixFromCamToOrigin, [[0, 0, 0, 1]]), 0)

# Camera Instantiation
cap = cv.VideoCapture(0)

while True:
    _, frame = cap.read()
    background = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # convert the background image to grayscale
    cv.imshow("colored_background", frame)
    cv.imshow("background_grayscale", background)

    esc = cv.waitKey(1)
    if esc == escapeKey:
        break

while True:
    _, frame = cap.read()
    foreground = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # convert the foreground image to grayscale
    cv.imshow("foreground", foreground)

    # Calculate the difference between the two images by substraction the foreground image from the background image in grayscale
    diff = np.absolute(np.matrix(np.int16(background)) - np.matrix(np.int16(foreground)))
    diff[diff > maxValueOfPixel] = maxValueOfPixel
    diff = np.uint8(diff)
    cv.imshow("diff", diff)

    # convert grayscale to black and white image 
    blackAndWhite = diff
    blackAndWhite[blackAndWhite <= threshold] = 0
    blackAndWhite[blackAndWhite > threshold] = 1

    # X cordinate calculation in pixel and converted to cm using the coordinate calculation algorithm in the camera frame
    xCord = np.matrix(np.sum(blackAndWhite, 0))
    xCordNumber = np.matrix(np.arange(640))
    xCordMultiply = np.multiply(xCord, xCordNumber)
    xTotal = np.sum(xCordMultiply)
    xTotal_total = np.sum(np.sum(blackAndWhite))
    xLocationInPixel = xTotal / xTotal_total
    xLocationInCm = xLocationInPixel * pixelToCm

    # Y cordinate calculation in pixel and converted to cm using the coordinate calculation algorithm in the camera frame
    yCordSum = np.matrix(np.sum(blackAndWhite, 1))
    yCord = yCordSum.transpose()
    yCordNumber = np.matrix(np.arange(480))
    yCordMultiply = np.multiply(yCord, yCordNumber)
    yTotal = np.sum(yCordMultiply)
    yTotal_total = np.sum(np.sum(blackAndWhite))
    yLocationInPixel = yTotal / yTotal_total
    yLocationInCm = yLocationInPixel * pixelToCm

    PointInCam = [[xLocationInCm], [yLocationInCm], [0], [1]] # put the cordinates into a vector 
    CordInBase = np.dot(HomogeneousMatrixFromCamToOrigin, PointInCam) # translates the coordinates of the image from camera frame into the base frame of the arm

    xCordInBase = CordInBase[0] # X coordinate in base frame
    yCordInBase = CordInBase[1] # Y coordinate in base frame

    print("the X coordinate is: %f and the Y coordinate is: %f" % (xCordInBase, yCordInBase)) # print the cordinates to the terminal

    esc = cv.waitKey(1) # exit when escape key is pressed
    if esc == escapeKey: 
        break

cv.destroyAllWindows()

# convert the coordinates from double into float value for transfare 
a = float(xCordInBase) 
b = float(yCordInBase)

serialPort = serial.Serial('COM8', 9600) # Open Serial Port with the necessary COM Port

serialPort.write(f"{a} {b}\n".encode()) # writes the coordinates to the microcontroler

serialPort.close() # close the Serial Port
