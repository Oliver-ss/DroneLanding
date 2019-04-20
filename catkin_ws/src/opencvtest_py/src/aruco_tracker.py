import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import socket

#ser = serial.Serial('/dev/ttyUSB0')

print("I'm working...") 

#TCP_IP = '169.254.137.76'
TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

print("I'm connected...") 



cap = cv2.VideoCapture(0)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('calib_images/*.jpg')


for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    #print(corners)
    #print(objpoints)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)


while (True):
    ret, frame = cap.read()
    # operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    
   # print(ids)

    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    if np.all(ids != None):
        rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
        #(rvec-tvec).any() # get rid of that nasty numpy value array error

        topleftX = corners[0][0][0][0]
        topleftY = corners[0][0][0][1]

        toprightX = corners[0][0][1][0]
        toprightY = corners[0][0][1][1]

        bottomleftX = corners[0][0][2][0]
        bottomlextY = corners[0][0][2][1]

        bottomrightX = corners[0][0][3][0]
        bottomrightY = corners[0][0][3][1]

        distance = tvec[0][0][2]


        print("topleft  corner x {}".format(topleftX))
        print("topleft corner y {}".format(topleftY))

        print("topright corner x {}".format(toprightX))
        print("topright corner y {}".format(toprightY))

        print("bottomleft corner x {}".format(bottomleftX))
        print("bottomleft corner y {}".format(bottomlextY))

        print("bottomright corner x {}".format(bottomrightX))
        print("bottomright corner y {}".format(bottomrightY))

        print("distance {}".format(distance))
        
        '''
        s.send(b"/tly", {}.format(topleftX))
        s.send(b"/tlx", "{}".format(topleftY))

        s.send(b"/trx", '{}'.format(toprightX))
        s.send(b"/try", '{}'.format(toprightY))

        s.send(b"/blx", '{}'.format(bottomleftX))
        s.send(b"/bly", '{}'.format(bottomlextY))

        s.send(b"/brx", '{}'.format(bottomrightX))
        s.send(b"/bry", '{}'.format(bottomrightY))
        '''

        midpointX = (topleftX  + bottomrightX)/2 
        midpointY = (topleftY + bottomrightY)/2

        

        print("midpoint X: {}, Y: {}".format(midpointX, midpointY))
        s.send("{}/{}/{}".format(midpointX, midpointY, distance).encode())


        aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1) #Draw Axis
        aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers


        ###### DRAW ID #####
        cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
    else:
        print("midpoint X: n, Y: n")
        s.send(b"/n/n/n")



        # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()