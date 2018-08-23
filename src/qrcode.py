import cv2
import cv2.aruco as aruco

cap = cv2.VideoCapture(0)

if __name__ == '__main__':
    while (True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        cap.set(3, 1280)
        cap.set(4, 720)

        # print(frame.shape) #480x640
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow('frame', gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        # Camera parameters
        f_x, f_y, c_x, c_y = 700.457, 700.457, 645.038, 361.34
        X_real, Y_real = 0.24, 0.202

        # lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # print(corners)
        # print(ids)
        max_area, max_id, max_indx = 5000, 0, 0
        center_x, center_y, width, height = 0, 0, 0, 0
        max_x, min_x, max_y, min_y = 0, 10000, 0, 10000

        print corners

        if len(corners) == 0:
            continue

        #print(corners)
        if_found_qr = 0

        for i in range(len(corners)):
            max_x, min_x, max_y, min_y = 0, 10000, 0, 10000
            for j in range(4):
                if(corners[i][0][j][0] > max_x):
                    max_x = corners[i][0][j][0]
                elif(corners[i][0][j][0] <= min_x):
                    min_x = corners[i][0][j][0]
                if (corners[i][0][j][1] > max_y):
                    max_y = corners[i][0][j][1]
                elif (corners[i][0][j][1] <= min_y):
                    min_y = corners[i][0][j][1]

            area = (max_x - min_x) * (max_y - min_y)
            print "area " + str(area)
            if area > max_area:
                max_area = area
                max_id = ids[i]
                max_index = i
                if_found_qr = 1

        if if_found_qr == 0:
            continue

        max_x, min_x, max_y, min_y = 0, 10000, 0, 10000

        for j in range(4):
            if (corners[max_index][0][j][0] > max_x):
                max_x = corners[max_index][0][j][0]
            elif (corners[max_index][0][j][0] <= min_x):
                min_x = corners[max_index][0][j][0]
            if (corners[max_index][0][j][1] > max_y):
                max_y = corners[max_index][0][j][1]
            elif (corners[max_index][0][j][1] <= min_y):
                min_y = corners[max_index][0][j][1]

        center_x = (max_x + min_x) / 2
        center_y = (max_y + min_y) / 2
        width = max_x - min_x
        height = max_y - min_y

        print "max_x, max_y: " + str(max_x) + str(max_y)
        print "width " + str(width)
        print "center_x " + str(center_x)
        Z = X_real / width * f_x
        X_real = (center_x - c_x) / f_x * Z
        Y_real = (center_y - c_y) / f_y * Z

        gray = aruco.drawDetectedMarkers(gray, corners)
        print(max_id)
        print(X_real, Y_real, Z)
        print("****************")
        # print(rejectedImgPoints)
        # Display the resulting frame


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()