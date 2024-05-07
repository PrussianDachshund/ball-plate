import time
import csv
import numpy as np
import cv2 as cv
import keyboard
import serial.tools.list_ports

# Platform parameters
r_base = 0.11154    # [m]
r_platform = 0.117  # [m]
servo_horn = 0.042  # [m]
link_sh_p = 0.0659  # [m]
z_home = 0.09171    # [m]
init_translation_vector = np.array([0, 0, 0])
angle_out_max = 8
angle_out_min = -8

# Serial parameters
port = 'COM5'
b_rate = 115200  # 9600

# PID Variables
error_x, error_y = 0, 0
pre_error_x, pre_error_y = 0, 0
# Romb
# Kp_x, Kp_y = 0.08, 0.1  # 0.1655, 0.1655  # 1.37904*0.12
# Kd_x, Kd_y = 3.5, 3.5 # (0.596785)/(0.05)*5.5 # 10 10
# pid_in_max, pid_in_min = 4.194, -4.194 #4.194
# Okrąg
# Kp_x, Kp_y = 0.00545, 0.0055  # 0.1655, 0.1655  # 1.37904*0.12
# Kd_x, Kd_y = 10.9, 11.9  # (0.596785)/(0.05)*5.5 # 10 10
# pid_in_max, pid_in_min = 4.194, -4.194  # 4.194

Kp_x, Kp_y = 0.08, 0.08  # 0.1655, 0.1655  # 1.37904*0.12
Kd_x, Kd_y = 81, 8   # (0.596785)/(0.05)*5.5 # 10 10
pid_in_max, pid_in_min = 4.194, -4.194  # 4.194

# LQR Variables
# K_mtx = np.array([[-1, -0.05792, 0, 0], [0, 0, -1, -0.05792]])
# K_mtx = np.array([[-0.7071, -0.7758, 0, 0], [0, 0, -0.7071, -0.7785]])
K_mtx = np.array([[-0.7071/1.3, -0.6521/3.5, 0, 0], [0, 0, -0.7071/1.3, -0.6521/3.5]])  # Dobry skok
# K_mtx = np.array([[-0.7071/1.5, -0.6521/4, 0, 0], [0, 0, -0.7071/1.5, -0.6521/5]]) # Spoko kwadrat
# K_mtx = np.array([[-1.1041/3, -0.3474/2.5, 0, 0], [0, 0, -1.1041/3.8, -0.3474/2.85]])  # Fajne koło
K_mtx = np.array([[-0.09792/2, -0.09132/10, 0, 0], [0, 0, -0.09545/8, -0.08617/12]])

Sampling_time = 0.01
lqr_in_max, lqr_in_min = 1, -1  # 1 -1 # 6303
pre_pos_x, pre_pos_y = 0, 0
#
# SP_x = 0  # SET_POINT (SP) IN X AXIS IN M
# SP_y = 0  # SET_POINT (SP) IN Y AXIS IN M

t = np.arange(0, 96.00, 0.1)  # TIME


# TRAJECTORIES
def step_to_zero():
    set_point_x = 0
    set_point_y = 0
    return set_point_x, set_point_y


# SQUARE TRAJECTORY
t1 = np.arange(0, 16.00, 0.1)
# sin = 0.1*np.sin(1.1*t)
sqr_x = np.zeros(len(t1))
sqr_x[0:20] = 0.07
for i in range(20, 40):  # ramp down
    sqr_x[i] = -0.07*(t1[i]-2)+0.07
sqr_x[40:60] = -0.07
for i in range(60, 80):  # ramp up
    sqr_x[i] = 0.07*(t1[i]-6)-0.07
sqr_x[80:100] = 0.07
for i in range(100, 120):  # ramp down
    sqr_x[i] = -0.07*(t1[i]-10)+0.07
sqr_x[120: 140] = -0.07
for i in range(140, 160):  # ramp up
    sqr_x[i] = 0.07*(t1[i]-14)-0.07

sqr_y = np.zeros((len(t1)))
for i in range(0, 20):  # ramp down
    sqr_y[i] = -0.07*(t1[i])+0.07
sqr_y[20:40] = -0.07
for i in range(40, 60):  # ramp up
    sqr_y[i] = 0.07*(t1[i]-4)-0.07
sqr_y[60:80] = 0.07
for i in range(80, 100):  # ramp down
    sqr_y[i] = -0.07*(t1[i]-8)+0.07
sqr_y[100:120] = -0.07
for i in range(120, 140):  # ramp up
    sqr_y[i] = 0.07*(t1[i]-12)-0.07
sqr_y[140:160] = 0.07

sqr_x = np.tile(sqr_x, 5)
sqr_y = np.tile(sqr_y, 5)
# CIRCLE TRAJECTORY
sin_x = 0.08*np.sin(1.8*t)
sin_y = 0.08*np.sin(1.8*t+np.pi/2)


# MAPPING CV TO TILT ANGLE
def cv_to_angle(control_output, in_min, in_max, out_min, out_max):
    angle = (control_output - in_min) * (out_max - out_min)/(in_max - in_min) + out_min
    if angle > 8:
        return 8
    elif angle < -8:
        return -8
    else:
        return angle


# CAMERA FUNCTIONS
def load_cal_param(file_name):  # Loads camera parameters for calibration
    return np.load('cam_data/' + str(file_name) + '.npy')


def load_param_to_var():  # Loading parameters for calibration to variables
    returned_value = load_cal_param('ret')
    camera_matrix = load_cal_param('mtx')
    distortion_coefficients = load_cal_param('distortion')
    rotation_vector = load_cal_param('r_vec')
    translation_vector = load_cal_param('t_vec')
    return returned_value, camera_matrix, distortion_coefficients, rotation_vector, translation_vector


def new_camera_matrix():  # Calculate new camera matrix and region of interest
    img = cv.imread('zdj/6.jpg')
    height, width = img.shape[:2]
    new_cam_mtx, region_of_interest = cv.getOptimalNewCameraMatrix(mtx, distortion, (width, height), 1, (width, height))
    return new_cam_mtx, region_of_interest


def cam_res_set(capture):
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)


def plate_detection(grey_cam_frame, color_cam_frame, text_font):
    circle_detect = \
        cv.HoughCircles(grey_cam_frame, cv.HOUGH_GRADIENT, 1, grey_cam_frame.shape[0], param1=80, param2=30,
                        minRadius=200, maxRadius=300)
    if circle_detect is not None:
        circle_detect = np.uint16(np.around(circle_detect))
        for i in circle_detect[0, :]:
            plt_center = (i[0], i[1])
            radius = i[2]
            cv.circle(color_cam_frame, plt_center, radius, (0, 255, 255), 2)
            cv.circle(color_cam_frame, plt_center, 0, (0, 255, 255), 2)
            cv.putText(img_cropped, (str(plt_center)), (plt_center[0] + 10, plt_center[1]),
                       text_font, 0.4, (0, 255, 255), 1, cv.LINE_AA)
            # print("circle: ", plt_center, radius)
            plate_center = plt_center
            return plate_center, radius
    return 1, 1


def ball_detection(color_cam_frame, text_font, plate_cent, plt_rad):
    hsv = cv.cvtColor(color_cam_frame, cv.COLOR_BGR2HSV)
    threshold = cv.inRange(hsv, (5, 21, 192), (29, 255, 255))
    contours, hierarchy = cv.findContours(threshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        ball = max(contours, key=cv.contourArea)
        moment = cv.moments(ball)
        if moment['m00'] != 0:
            center = (int(moment['m10'] / moment['m00']), int(moment['m01'] / moment['m00']))
        else:
            center = (0, 0)
        pos_coff = 0.270/(plt_rad*2)
        regulation_position = np.array([round(((center[0] - plate_cent[0]) * pos_coff), 4),
                                        round(((center[1] - plate_cent[1]) * pos_coff), 4)])  # [M]
        ball_cords_x = (str(round(((center[0] - plate_cent[0]) * pos_coff*1000), 2)), 'mm')
        ball_cords_y = (str(round(((center[1] - plate_cent[1]) * pos_coff*1000), 2)), 'mm')
        ball_cords_str = (' '.join(ball_cords_x), ' '.join(ball_cords_y))
        cv.circle(color_cam_frame, (center[0], center[1]), 5, (255, 255, 255), -1)
        cv.putText(color_cam_frame, str(ball_cords_str), (center[0]+6, center[1]), text_font,
                   0.4, (255, 255, 255), 1, cv.LINE_AA)
        # print('Ball cords: ', center)
        # print('BALLS: ', regulation_position)
        ball_found = 1
        return ball_found, regulation_position, center
    else:
        print("Ball not found!")
        ball_found = 0
        regulation_position = np.array([0, 0])
        return ball_found, regulation_position, 1


# INVERSE KINEMATICS FUNCTIONS

def base_coordinates_matrix(base_radius):
    base_joints_deg = np.array([90-5.8, 210-5.8, 330-5.8])
    base_joints_rad = np.radians(base_joints_deg)
    base_position = np.zeros((3, 3))
    for num, rad in enumerate(base_joints_rad):
        base_position[0, num] = base_radius*np.cos(rad)
        base_position[1, num] = base_radius*np.sin(rad)
        base_position[2, num] = 0
    return base_position


def platform_coordinates_matrix(platform_radius, height):
    platform_joint_deg = np.array([90, 210, 330])
    platform_joint_rad = np.radians(platform_joint_deg)
    platform_position = np.zeros((3, 3))
    for num, rad in enumerate(platform_joint_rad):
        platform_position[0, num] = platform_radius*np.cos(rad)
        platform_position[1, num] = platform_radius*np.sin(rad)
        platform_position[2, num] = height
    return platform_position


def legs_vector(rotation_vector, translation_vector, platform_joints, base_joints):
    rotation_vector = np.radians(rotation_vector)
    a, b = rotation_vector
    rotation_matrix = np.array([
        [np.cos(a), np.sin(a)*np.sin(b), np.cos(b)*np.sin(b)],
        [0, np.cos(b), -np.sin(b)],
        [-np.sin(a), np.cos(a)*np.sin(b), np.cos(b)*np.cos(a)]
    ])
    translation_matrix = np.tile(translation_vector.reshape(3, 1), (1, 3))

    virtual_lengths = np.subtract(np.add(translation_matrix, np.dot(rotation_matrix, platform_joints)), base_joints)
    return virtual_lengths


def legs_length(leg_lengths_vector):
    transpose_virtual_lengths = np.transpose(leg_lengths_vector)
    leg_vector_length = np.array([np.linalg.norm(transpose_virtual_lengths[0]),
                                  np.linalg.norm(transpose_virtual_lengths[1]),
                                  np.linalg.norm(transpose_virtual_lengths[2])])
    return leg_vector_length


def calculate_servo_angle(leg_vector_length, servo_horn_length, link_length, platform_joints, base_joints):
    servo_angles = np.zeros(3)
    servo_distribution_deg = np.array([84.24, 204.24, 324.24])
    servo_distribution_rad = np.radians(servo_distribution_deg)
    for num, (deg, length) in enumerate(zip(servo_distribution_rad, leg_vector_length)):
        a = 2 * servo_horn_length * (platform_joints[2, num] - base_joints[2, num])
        b = 2 * servo_horn_length * (np.cos(deg)*(platform_joints[0, num]-base_joints[0, num]) + np.sin(deg) *
                                     (platform_joints[1, num]-base_joints[1, num]))
        c = abs(length*length) - link_length*link_length + servo_horn_length*servo_horn_length
        temp = c/(np.sqrt(a*a + b*b))
        if temp >= 1:
            temp = 0.99
        servo_angles[num] = np.arcsin(temp) - np.arctan(b/a)
    servo_angles = np.degrees(servo_angles)

    return servo_angles


# CAMERA VARIABLES
ret, mtx, distortion, r_vec, t_vec = load_param_to_var()
new_camera_mtx, roi = new_camera_matrix()
cap = cv.VideoCapture(0)
cam_res_set(cap)
font = cv.FONT_HERSHEY_SIMPLEX

# INVERSE KINEMATICS INIT
base_pos = base_coordinates_matrix(r_base)
plate_pos = platform_coordinates_matrix(r_platform, z_home)

state = 3  # 0 - pause in default state, 1 - PD regulation, 2 - LQR - regulation, 3 - pause in actual state
index = 0
timer = 0
#f = open('PD_reg_kw.csv', 'w', newline='')
f2 = open('LQR_reg.csv', 'w', newline='')
fieldnames = ['time', 'pv_x', 'pv_y', 'error_x', 'error_y', 'spx', 'spy']
#the_writer = csv.DictWriter(f, fieldnames=fieldnames)
the_writer2 = csv.DictWriter(f2, fieldnames=fieldnames)
the_writer2.writeheader()

# MAIN
while True:
    ret_val_cap, frame = cap.read()
    dst_color = cv.undistort(frame, mtx, distortion, None, new_camera_mtx)  # Undistorted frame captured by camera
    x, y, w, h = roi
    img_cropped = dst_color[500 - 265:500 + 265, 1055 - 265:1055 + 265]  # Crops frame to only plate view
    img_cropped = cv.bilateralFilter(img_cropped, 9, 75, 75)
    img_cropped_grey = cv.cvtColor(img_cropped, cv.COLOR_BGR2GRAY)
    plate_ctr, plate_radius = plate_detection(img_cropped_grey, img_cropped, font)
    bf, ball_pos, ball_pos_px = ball_detection(img_cropped, font, plate_ctr, plate_radius)

    # STATE 0 - RESET TO DEFAULT POSITION
    if state == 0:
        print('State: Reset to default position. Ball found:', bf)
        cv.putText(img_cropped, 'IK mode', (10, 25), font, 0.6, (0, 0, 0), 1, cv.LINE_AA)
        rot_vector = np.array([0, 0])
        legs = legs_vector(rot_vector, init_translation_vector, plate_pos, base_pos)
        leg_length = legs_length(legs)
        angles = calculate_servo_angle(leg_length, servo_horn, link_sh_p, plate_pos, base_pos)
        angles = np.around(angles, 2)
        servo1_angle = angles[0]
        servo2_angle = angles[1]
        servo3_angle = angles[2]
        print('Angles: ', str(str(angles)+'\n'))
        with serial.Serial(port, b_rate, timeout=10) as ser:
            ser.write(str(str(angles)+'\n').encode())
            print(str(str(angles)+'\n').encode())
            s1 = ser.readline()
            s2 = ser.readline()
            s3 = ser.readline()
            s4 = ser.readline()
            s5 = ser.readline()
            s6 = ser.readline()
            s7 = ser.readline()
            s1_dec = s1.decode()
            s2_dec = s2.decode()
            s3_dec = s3.decode()
            s4_dec = s4.decode()
            s5_dec = s5.decode()
            s6_dec = s6.decode()
            print('Arduino angles: ', s1_dec, s2_dec, s3_dec, s4_dec, s5_dec, s6_dec, s7)
        time.sleep(Sampling_time)
    # STATE 1 - PD REGULATION
    elif state == 1:
        cv.putText(img_cropped, 'PD regulation', (10, 25), font, 0.6, (0, 0, 0), 1, cv.LINE_AA)
        if bf == 1:
            pos_x, pos_y = ball_pos
            if pow((np.float32(ball_pos_px[0]) - np.float32(plate_ctr[0])), 2) + \
                    pow((np.float32(ball_pos_px[1]) - np.float32(plate_ctr[1])), 2) \
                    <= np.float32(pow(plate_radius, 2)):
                print('State: PD regulation. Ball found:', bf)
                spx = sqr_x[index]
                spy = sqr_y[index]
                print('SP: ', spx, spy, index)
                error_x = spx - pos_x
                error_y = spy - pos_y
                proportional_x = error_x*Kp_x
                proportional_y = error_y*Kp_y
                derivative_x = Kp_x*Kd_x*((error_x - pre_error_x)/Sampling_time)
                derivative_y = Kp_y*Kd_y*((error_y - pre_error_y)/Sampling_time)
                output_x = proportional_x + derivative_x
                output_y = proportional_y + derivative_y
                roll = cv_to_angle(output_x, pid_in_min, pid_in_max, angle_out_min, angle_out_max)
                pitch = cv_to_angle(output_y, pid_in_min, pid_in_max, angle_out_min, angle_out_max)
                pre_error_x = error_x
                pre_error_y = error_y

                rot_vector = np.array([-roll, -pitch])
                legs = legs_vector(rot_vector, init_translation_vector, plate_pos, base_pos)
                leg_length = legs_length(legs)
                angles = calculate_servo_angle(leg_length, servo_horn, link_sh_p, plate_pos, base_pos)
                angles = np.around(angles, 2)
                servo1_angle = angles[0]
                servo2_angle = angles[1]
                servo3_angle = angles[2]  # INVERSE KINEMATICS
                print("Angles: ", roll, pitch, " Pos: ", pos_x, pos_y, " Error: ", error_x, error_y, "SP: ", spx, spy)
                # SERIAL WRITE ANGLES
                with serial.Serial(port, b_rate, timeout=10) as ser:  # SERIAL WRITE ANGLES
                    ser.write(str(str(angles) + '\n').encode())
                    s1 = ser.readline()
                    s2 = ser.readline()
                    s3 = ser.readline()
                    s4 = ser.readline()
                    s5 = ser.readline()
                    s6 = ser.readline()
                    s7 = ser.readline()
                    s1_dec = s1.decode()
                    s2_dec = s2.decode()
                    s3_dec = s3.decode()
                    s4_dec = s4.decode()
                    s5_dec = s5.decode()
                    s6_dec = s6.decode()
                    # print('Arduino angles: \n', s1_dec, s2_dec, s3_dec, s4_dec, s5_dec, s6_dec, s7)  # #
                # if index < len(t):
                #     # the_writer.writerow({'time': t[index], 'pv_x': pos_x, 'pv_y': pos_y, 'error_x': error_x,
                #     #                      'error_y': error_y, 'spx': spx, 'spy': spy})
                #     # index = index + 1
                time.sleep(Sampling_time)

        elif bf == 0:
            print('Ball not found! Put ball back on plate, kind sir.')

    # STATE 2 - LQR REGULATION
    elif state == 2:
        cv.putText(img_cropped, 'LQR regulation', (10, 25), font, 0.6, (0, 0, 0), 1, cv.LINE_AA)
        if bf == 1:
            pos_x, pos_y = ball_pos
            if pow((np.float32(ball_pos_px[0])-np.float32(plate_ctr[0])), 2) +\
                    pow((np.float32(ball_pos_px[1])-np.float32(plate_ctr[1])), 2) \
                    <= np.float32(pow(plate_radius, 2)):
                print('State: LQR regulation. Ball found:', bf)
                dot_pos_x = (pos_x - pre_pos_x)/Sampling_time
                dot_pos_y = (pos_y - pre_pos_y)/Sampling_time
                #SP_x = sqr_x[index]
                #SP_y = sqr_y[index]
                SP_x = 0
                SP_y = 0
                print('SP: ', SP_x, SP_y)
                state_matrix = np.array([[pos_x], [dot_pos_x], [pos_y], [dot_pos_y]])
                state_lqr_feedback = np.dot(-K_mtx, state_matrix)
                # print(state_lqr_feedback)
                set_point_mtx = np.array([[SP_x], [SP_y]])
                control_signal = np.add(state_lqr_feedback, set_point_mtx)
                ctr_x = np.float64(control_signal[0])
                ctr_y = np.float64(control_signal[1])
                pre_pos_x = pos_x
                pre_pos_y = pos_y
                roll = cv_to_angle(ctr_x, lqr_in_min, lqr_in_max, angle_out_min, angle_out_max)
                pitch = cv_to_angle(ctr_y, lqr_in_min, lqr_in_max, angle_out_min, angle_out_max)
                print("Angles: ", roll, pitch, " Pos: ", pos_x, pos_y, " Error: ", control_signal, sin_x[index])
                rot_vector = np.array([roll, pitch])
                legs = legs_vector(rot_vector, init_translation_vector, plate_pos, base_pos)
                leg_length = legs_length(legs)
                angles = calculate_servo_angle(leg_length, servo_horn, link_sh_p, plate_pos, base_pos)
                angles = np.around(angles, 2)
                servo1_angle = angles[0]
                servo2_angle = angles[1]
                servo3_angle = angles[2]
                print(str(str(angles)+'\n'))
                with serial.Serial(port, b_rate, timeout=10) as ser:
                    ser.write(str(str(angles) + '\n').encode())
                    s1 = ser.readline()
                    s2 = ser.readline()
                    s3 = ser.readline()
                    s4 = ser.readline()
                    s5 = ser.readline()
                    s6 = ser.readline()
                    s7 = ser.readline()
                    s1_dec = s1.decode()
                    s2_dec = s2.decode()
                    s3_dec = s3.decode()
                    s4_dec = s4.decode()
                    s5_dec = s5.decode()
                    s6_dec = s6.decode()
                    # print('Arduino angles: \n', s1_dec, s2_dec, s3_dec, s4_dec, s5_dec, s6_dec, s7)
                if index < len(t):
                    the_writer2.writerow({'time': t[index], 'pv_x': pos_x, 'pv_y': pos_y, 'error_x': SP_x-pos_x,
                                         'error_y': SP_y-pos_y, 'spx': SP_x, 'spy': SP_y})
                    index = index + 1
                time.sleep(Sampling_time)

        elif bf == 0:
            print('Ball not found! Put ball back on plate, kind sir.')

    elif state == 3:
        print('State: Pause in actual state. Ball found:', bf)
        cv.putText(img_cropped, 'Pause', (10, 25), font, 0.6, (0, 0, 0), 1, cv.LINE_AA)

    cv.imshow('Window, combined', img_cropped)  # Window display

    # SIMPLE STATE MACHINE
    if keyboard.is_pressed('1'):
        state = 0
    if keyboard.is_pressed('2'):
        state = 1
    if keyboard.is_pressed('3'):
        state = 2
    if keyboard.is_pressed('4'):
        state = 3

    # QUIT & RETURN TO DEFAULT POSITION
    if cv.waitKey(1) == ord('q'):
        break

f2.close()
cap.release()
ser.close()
cv.destroyAllWindows()
