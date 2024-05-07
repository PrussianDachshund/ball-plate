import cv2 as cv
import numpy as np
import glob


def save_cam_parameters(returned_value, camera_matrix, distortion_params, rot_vector, trans_vector):
    np.save("cam_data/ret.npy", returned_value)
    np.save("cam_data/mtx.npy", camera_matrix)
    np.save("cam_data/distortion", distortion_params)
    np.save("cam_data/r_vec.npy", rot_vector)
    np.save("cam_data/t_vec.npy", trans_vector)


Calibration_board = (9, 6)

iter_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

real_world_cords = np.zeros((Calibration_board[0] * Calibration_board[1], 3), np.float32)
real_world_cords[:, :2] = np.mgrid[0:Calibration_board[0], 0:Calibration_board[1]].T.reshape(-1, 2)

cords3d = []
cords2d = []

images = glob.glob('zdj/*.jpg', recursive=True)
i = 1
for image in images:
    # print(image)
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, Calibration_board, None)

    if ret:
        cords3d.append(real_world_cords)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), iter_criteria)
        cords2d.append(corners)
        img2 = cv.drawChessboardCorners(img, Calibration_board, corners2, ret)
        cv.imshow('img2', img2)
        print(i)
        cv.imwrite('zdj/1'+str(i)+'.jpg', img2)
        i = i + 1
        cv.waitKey(500)
cv.destroyAllWindows()

ret, mtx, distortion, r_vec, t_vec = cv.calibrateCamera(cords3d, cords2d, gray.shape[::-1], None, None)
save_cam_parameters(ret, mtx, distortion, r_vec, t_vec)
print('Camera Calibrated: ', ret)
print('\nCamera Matrix:\n', mtx)
print('\nDistortion:\n', distortion)
print('\nRotation:\n', r_vec)
print('\nTranslation:\n', t_vec)


