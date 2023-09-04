import serial
import time
import matplotlib.pyplot as plt

class Connection_Azurre():
    def __init__(self, *args):
        self.ser = serial.Serial()
        self.ser.baudrate = args[0]
        self.ser.port = args[1]
        self.ser.open()
    def close_conn(self):
        self.ser.close()
    def refresh_conn(self):
        self.ser.flush()
class Serial_pck():
    def __init__(self, *args):
        self.in_bytes = args[0] # List of numbers to be transmitted
        self.type = args[1] # It can be hex, dec, or bit
        self.out_bytes = None
        self.n_bytes = len(self.in_bytes) # Number of bytes to send to the hand
        if self.type == 'bin':
            self.out_bytes = bytearray([int(x, 2) for x in [str(x) for x in self.in_bytes]])
        else:
            self.out_bytes = bytearray(self.in_bytes)
            
class Handler():
    def __init__(self, conn):
        self.conn = conn
    def send_pck(self, tx_bytes):
        return self.conn.ser.write(tx_bytes)
    def get_pck(self, rx_bytes):
        # rx_bytes contains the number of bytes to wait for
        return self.conn.ser.read(rx_bytes)

class ih2_azurra():
    def __init__(self, handler):
        # Order of fingers are: Thumb abd/add, Thumb fl/ex, Index fl/ex, Middle fl/ex, Ring_Little fl/ex
        self.send_cmds = {'GetFingerPosition': [0x45,[0x0,0x1,0x2,0x3,0x4]],
                          'GetFingerStatus': [0x4b, [0x0, 0x1, 0x2, 0x3, 0x4]],
                          'GetFingerForce': [0x0,0x1,0x2,0x3,0x4],
                          'GetMotorCurrent': [0x49, [0x0, 0x1, 0x2, 0x3, 0x4]],
                          'SetHandPosture': [0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48],
                          'SetFingerPosition': [0x44, 0x00, 0x00],
                          'Automatic_grasps': [[0x6f, 0x00, 0x00, 0x48], {'Cylindrical': 4,
                                                                          'Tri-digit': 3,
                                                                          'Bi-digit': 2,
                                                                          'Lateral': 1,
                                                                          'Tri-digit-ext': 31,
                                                                          'Bi-digit-ext': 21,
                                                                          'Buffet': 11,
                                                                          'Three': 6,
                                                                          'Pistol': 7,
                                                                          'Thumb-up': 8,
                                                                          'Relax': 0}],
                          'OpenALL': [0x4c],
                          'StopALL': [0x41],
                          'OpenMotor': [[10000001, 11111111], [10000101, 11111111], [10001001, 11111111], [10001101, 11111111], [10010001, 11111111]],
                          'CloseMotor': [[11000001, 11111111], [11000101, 11111111], [11001001, 11111111], [11001101, 11111111], [11010001, 11111111]],
                          'FastCalibration': [0x46],
                          'SetFingerForce': [0x4a, 0x00, 0x00],
                          'SetFingerCurrent': [0x5f, 0x00, 0x61, 0x00, 0x00, 0x00]}
        self.handler = handler

    def get_finger_position(self, finger):
        pck = [self.send_cmds['GetFingerPosition'][0], self.send_cmds['GetFingerPosition'][1][finger]]
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        rx = self.handler.get_pck(1)
        return pck, int.from_bytes(rx, "big")
    def get_finger_status(self, finger):
        pck = [self.send_cmds['GetFingerStatus'][0], self.send_cmds['GetFingerStatus'][1][finger]]
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        rx = self.handler.get_pck(1)
        rx = int.from_bytes(rx, "big")
        return pck, format(rx, '#010b')[2:]
    def get_finger_force(self, finger):
        pck = [self.send_cmds['GetFingerForce'][finger]]
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        rx = self.handler.get_pck(2)
        rx = int.from_bytes(rx, "big")
        return pck, format(rx, '#018b')[2:]
    def get_motor_current(self, finger):
        pck = [self.send_cmds['GetMotorCurrent'][0], self.send_cmds['GetMotorCurrent'][1][finger]]
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        rx = self.handler.get_pck(2)
        rx = int.from_bytes(rx, "big")
        return pck, format(rx, '#018b')[2:]
    def set_hand_posture(self, pos, type):
        if type == 'bin':
            pos = [int(x, 2) for x in [str(x) for x in pos]]
        self.send_cmds['SetHandPosture'][1:-1] = pos
        pck = self.send_cmds['SetHandPosture']
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        return pck
    def set_finger_position(self, doa, pos, type):
        if type == 'bin':
            doa = int(str(doa), 2)
            pos = int(str(pos), 2)
        self.send_cmds['SetFingerPosition'][1] = doa
        self.send_cmds['SetFingerPosition'][2] = pos
        pck = self.send_cmds['SetFingerPosition']
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        return pck
    def set_automatic_grasp(self, grasp, f, t):
        if type == 'bin':
            f = int(str(f), 2)
            t = int(str(t), 2)
        self.send_cmds['Automatic_grasps'][0][1] = self.send_cmds['Automatic_grasps'][1][grasp]
        self.send_cmds['Automatic_grasps'][0][2] = f
        self.send_cmds['Automatic_grasps'][0][3] = t
        pck = self.send_cmds['Automatic_grasps'][0]
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        return pck
    def open_all(self):
        pck = self.send_cmds['OpenALL']
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        return pck
    def stop_all(self):
        pck = self.send_cmds['StopALL']
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        return pck
    def move_motor_full_speed(self, motor, opcl):
        if opcl == 0:
            pck = self.send_cmds['OpenMotor'][motor]
        else:
            pck = self.send_cmds['CloseMotor'][motor]
        self.handler.send_pck(Serial_pck(pck, 'bin').out_bytes)
        return pck
    def set_finger_force(self, finger, force):
        force = format(force, '#012b')[2:]
        self.send_cmds['SetFingerForce'][1] = int(force[0], base = 2)*128 + int(force[1], base = 2)*64 + finger
        self.send_cmds['SetFingerForce'][2] = int(force[2:], base = 2)
        pck = self.send_cmds['SetFingerForce']
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        return pck
    def set_finger_current(self, finger, curr):
        curr = format(curr, '#012b')[2:]
        self.send_cmds['SetFingerCurrent'][1] = finger
        self.send_cmds['SetFingerCurrent'][3] = int(curr[0], base=2) * 2 + int(curr[1], base=2) * 1
        self.send_cmds['SetFingerCurrent'][4] = int(curr[2:], base=2)
        self.send_cmds['SetFingerCurrent'][5] = finger
        pck = self.send_cmds['SetFingerCurrent']
        self.handler.send_pck(Serial_pck(pck, 'hex').out_bytes)
        return pck
        
class controller_azurra():
    def __init__(self, ih2, type, var):
        # Receives as an input an object from the class azurra
        self.ih2 = ih2
        self.type = type
        self.var = var
        self.status = False # Boolean indicating whether the
        self.error = []
        self.t = []
        self.response = []
        self.sets = []
        self.dt = 0.01
        self.errorPlot = None
        self.respPlot = None
        self.ax = None
    def control_finger(self, finger, set_p):
        self.error = []
        self.t = []
        self.response = []
        self.sets = []
        cnt_success = 0
        c_s = self.ih2.get_finger_position(finger)[1] # Control signal initialization on the current pose
        set_p = self.__boundary_control(set_p, 0, 255)
        it = 0

        if self.type == 'P' and self.var == 'Position':
            kp = 0.065  # Proportional gain
            print('Controlling Position with P Controller')
            try:
                while True:
                    curr_var = self.ih2.get_finger_position(finger)[1]
                    if type(set_p) is int and it == 0:
                        self.sets.append(curr_var)
                    else:
                        self.sets.append(set_p)
                    print(c_s, curr_var, set_p)
                    if set_p - curr_var < 1:
                        cnt_success += 1
                    self.error.append(set_p - curr_var)
                    self.t.append(self.dt*it)
                    c_s = c_s + kp * (set_p - curr_var)
                    self.response.append(curr_var)
                    c_s = self.__boundary_control(c_s, 0, 255)
                    self.ih2.set_finger_position(finger, int(c_s), 'dec')
                    time.sleep(self.dt)
                    if cnt_success >= 50:
                        self.status = True
                    it += 1
            finally:
                self.display_error()
                self.display_response()

        elif self.type == 'PD' and self.var == 'Position':
            kp = 0.14 # Proportional gain
            kd = 0.005 # Derivative gain
            print('Controlling Position with PD Controller')
            try:
                while True:
                    curr_var = self.ih2.get_finger_position(finger)[1]
                    if type(set_p) is int and it == 0:
                        self.sets.append(curr_var)
                    else:
                        self.sets.append(set_p)
                    print(c_s, curr_var, set_p)
                    if set_p - curr_var < 1:
                        cnt_success += 1
                    self.t.append(self.dt * it)
                    curr_error = set_p - curr_var
                    self.error.append(curr_error)
                    c_s = [c_s + kp * (curr_error) + kd * ((curr_error - self.error[it - 1])/self.dt), c_s][it == 0]
                    self.response.append(curr_var)
                    c_s = self.__boundary_control(c_s, 0, 255)
                    self.ih2.set_finger_position(finger, int(c_s), 'dec')
                    time.sleep(self.dt)
                    if cnt_success >= 50:
                        self.status = True
                    it += 1
            finally:
                self.display_error()
                self.display_response()

        elif self.type == 'PID' and self.var == 'Position':
            kp = 0.1 # Proportional gain
            kd = 0.007# Derivative gain
            ki = 0.00005 # Integral gain
            print('Controlling Position with PID Controller')
            I = 0 # integral part
            try:
                while True:
                    curr_var = self.ih2.get_finger_position(finger)[1]
                    if type(set_p) is int and it == 0:
                        self.sets.append(curr_var)
                    else:
                        self.sets.append(set_p)
                    print(c_s, curr_var, set_p)
                    if set_p - curr_var < 1:
                        cnt_success += 1
                    self.t.append(self.dt * it)
                    curr_error = set_p - curr_var
                    self.error.append(curr_error)
                    I = I + ki * curr_error * (self.dt)
                    c_s = [c_s + kp * (curr_error) + kd * ((curr_error - self.error[it - 1])/self.dt) + I, c_s][it == 0]
                    self.response.append(curr_var)
                    c_s = self.__boundary_control(c_s, 0, 255)
                    self.ih2.set_finger_position(finger, int(c_s), 'dec')
                    time.sleep(self.dt)
                    if cnt_success >= 50:
                        self.status = True
                    it += 1
            finally:
                self.display_error()
                self.display_response()

        elif self.type == 'PID' and self.var == 'Current':
            kp = 3.0  # Proportional gain
            kd = 0.4  # Derivative gain
            ki = 0.05  # Integral gain
            print('Controlling Current with PID Controller')
            I = 0  # integral part
            c_s = 0
            try:
                while True:
                    curr_var = self.ih2.get_finger_position(finger)[1]
                    if type(set_p) is int and it == 0:
                        self.sets.append(curr_var)
                    else:
                        self.sets.append(set_p)
                    print(c_s, curr_var, set_p)
                    if set_p - curr_var < 1:
                        cnt_success += 1
                    self.t.append(self.dt * it)
                    curr_error = set_p - curr_var
                    self.error.append(curr_error)
                    I = I + ki * curr_error * (self.dt)
                    c_s = [c_s + kp * (curr_error) + kd * ((curr_error - self.error[it - 1]) / self.dt) + I, c_s][it == 0]
                    self.response.append(curr_var)
                    c_s = self.__boundary_control(c_s, 0, 1023)
                    self.ih2.set_finger_current(finger, int(c_s))
                    time.sleep(self.dt)
                    if cnt_success >= 50:
                        self.status = True
                    it += 1
            finally:
                self.display_error()
                self.display_response()

        elif self.type == 'PID' and self.var == 'Force':
            kp = 0.5  # Proportional gain
            kd = 0.06  # Derivative gain
            ki = 0.008  # Integral gain
            print('Controlling Force with PID Controller')
            I = 0  # integral part
            c_s = 0
            try:
                while True:
                    curr_var = self.ih2.get_finger_position(finger)[1]
                    if type(set_p) is int and it == 0:
                        self.sets.append(curr_var)
                    else:
                        self.sets.append(set_p)
                    print(c_s, curr_var, set_p)
                    if set_p - curr_var < 1:
                        cnt_success += 1
                    self.t.append(self.dt * it)
                    curr_error = set_p - curr_var
                    self.error.append(curr_error)
                    I = I + ki * curr_error * (self.dt)
                    c_s = [c_s + kp * (curr_error) + kd * ((curr_error - self.error[it - 1]) / self.dt) + I, c_s][it == 0]
                    self.response.append(curr_var)
                    c_s = self.__boundary_control(c_s, 0, 1023)
                    self.ih2.set_finger_force(finger, int(c_s))
                    time.sleep(self.dt)
                    if cnt_success >= 50:
                        self.status = True
                    it += 1
            finally:
                self.display_error()
                self.display_response()

        if self.status == True:
            print('Controller Successful')

    def control_grasp(self, set_p):
        self.error = [[] for i in range(5)]
        self.t = [[] for i in range(5)]
        self.response = [[] for i in range(5)]
        self.sets = [[] for i in range(5)] * 5
        cnt_success = [0 for i in range(5)]
        self.dt = 0.01
        c_s = []
        for i in range(5):
            c_s.append(self.ih2.get_finger_position(i)[1]) # Control signal initialization on the current poses of all motors
        if type(set_p) is not list or len(set_p) < 5:
            print('ERROR, Set points should be a 1x5 list')
            return
        set_p = self.__boundary_control(set_p, 0, 255) # Checking set points boundaries for all
        it = [0 for i in range(5)]

        if self.type == 'PID' and self.var == 'Current':
            kp = 6.0  # Proportional gain
            kd = 0.25  # Derivative gain
            ki = 0.1  # Integral gain
            print('Controlling Current with PID Controller')
            I = [0 for i in range(5)]  # integral part
            c_s = [0 for i in range(5)]
            try:
                while True:
                    for i in range(5):
                        curr_var = self.ih2.get_finger_position(i)[1]
                        if type(set_p) is list and it[i] == 0:
                            self.sets[i].append(curr_var)
                        else:
                            self.sets[i].append(set_p[i])
                        if set_p[i] - curr_var < 1:
                            cnt_success[i] += 1
                        curr_error = set_p[i] - curr_var
                        self.error[i].append(curr_error)
                        I[i] = I[i] + ki * curr_error * (self.dt)
                        c_s[i] = [c_s[i] + kp * (curr_error) + kd * ((curr_error - self.error[i][it[i] - 1]) / self.dt) + I[i], c_s[i]][it == 0]
                        self.response[i].append(curr_var)
                        c_s = self.__boundary_control(c_s, 0, 1023)
                        self.ih2.set_finger_current(i, int(c_s[i]))
                        if it[i] == 0:
                            self.t[i].append(self.dt * it[i] + 0.005 * 1)
                        else:
                            self.t[i].append(self.dt * it[i] + 0.005 * 5)
                        it[i] += 1
                    time.sleep(self.dt)
                    print(self.response[0][it[0] - 1],
                          self.response[1][it[1] - 1],
                          self.response[2][it[2] - 1],
                          self.response[3][it[3] - 1],
                          self.response[4][it[4] - 1])
                    if cnt_success >= [50,50,50,50,50]:
                        self.status = True
            finally:
                self.display_response_grasp()
                self.display_error_grasp()

        if self.type == 'PID' and self.var == 'Force':
            kp = 0.65  # Proportional gain
            kd = 0.02  # Derivative gain
            ki = 0.01  # Integral gain
            print('Controlling Force with PID Controller')
            I = [0 for i in range(5)]  # integral part
            c_s = [0 for i in range(5)]
            set_p[0] = 0 # Tendon force control is not possible for the abduction DoF. Attempting this can be problematic and corrupt bytes transmission.
            try:
                while True:
                    for i in range(5):
                        curr_var = self.ih2.get_finger_position(i)[1]
                        if type(set_p) is list and it[i] == 0:
                            self.sets[i].append(curr_var)
                        else:
                            self.sets[i].append(set_p[i])
                        if set_p[i] - curr_var < 1:
                            cnt_success[i] += 1
                        curr_error = set_p[i] - curr_var
                        self.error[i].append(curr_error)
                        I[i] = I[i] + ki * curr_error * (self.dt)
                        c_s[i] = [c_s[i] + kp * (curr_error) + kd * ((curr_error - self.error[i][it[i] - 1]) / self.dt) + I[i], c_s[i]][it == 0]
                        self.response[i].append(curr_var)
                        c_s = self.__boundary_control(c_s, 0, 1023)
                        self.ih2.set_finger_force(i, int(c_s[i]))
                        if it[i] == 0:
                            self.t[i].append(self.dt * it[i] + 0.005 * 1)
                        else:
                            self.t[i].append(self.dt * it[i] + 0.005 * 5)
                        it[i] += 1
                    time.sleep(self.dt)
                    print(self.response[0][it[0] - 1],
                          self.response[1][it[1] - 1],
                          self.response[2][it[2] - 1],
                          self.response[3][it[3] - 1],
                          self.response[4][it[4] - 1])
                    if cnt_success >= [50,50,50,50,50]:
                        self.status = True
            finally:
                self.display_response_grasp()
                self.display_error_grasp()
    def __boundary_control(self, var, l_b, h_b):
        if type(var) is int or type(var) is float:
            var = [var, l_b][var <= l_b]
            var = [var, h_b][var >= h_b]
        elif type(var) is list:
            for i in range(len(var)):
                var[i] = [var[i], l_b][var[i] <= l_b]
                var[i] = [var[i], h_b][var[i] >= h_b]
        return var

    def display_response_grasp(self):
        self.respPlot = plt.figure()
        for i in range(5):
            self.ax = self.respPlot.add_subplot(2, 3, i + 1)
            self.ax.plot(self.t[i], self.response[i], self.t[i], self.sets[i])
            self.ax.set_title('Response of DOF_' + str(i + 1) + " vs Time")
            self.ax.set_xlabel('Time [s]')
            self.ax.set_ylabel('Control Variable')
            self.ax.xaxis.grid(True)
            self.ax.yaxis.grid(True)
    def display_response(self):
        self.respPlot = plt.figure()
        self.ax = self.respPlot.add_subplot(1, 1, 1)
        self.ax.plot(self.t, self.response, self.t, self.sets)
        self.ax.set_title('Response vs Time')
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Control Variable')
        self.ax.xaxis.grid(True)
        self.ax.yaxis.grid(True)
    def display_error_grasp(self):
        self.errorPlot = plt.figure()
        for i in range(5):
            self.ax = self.errorPlot.add_subplot(2, 3, i + 1)
            self.ax.plot(self.t[i], self.error[i])
            self.ax.set_title('Error of DOF_' + str(i + 1) + " vs Time")
            self.ax.set_xlabel('Time [s]')
            self.ax.set_ylabel('Error')
            self.ax.xaxis.grid(True)
            self.ax.yaxis.grid(True)
    def display_error(self):
        self.errorPlot = plt.figure()
        self.ax = self.errorPlot.add_subplot(1, 1, 1)
        self.ax.plot(self.t, self.error)
        self.ax.set_title('Error vs Time')
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Error')
        self.ax.xaxis.grid(True)
        self.ax.yaxis.grid(True)
