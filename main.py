import time
from toolkits import *

baudrate = 115200
port = '/dev/ttyUSB1'

if __name__ == '__main__':
    print('---- Stablishing Connection ----')
    conn_az = Connection_Azurre(baudrate, port)
    h = Handler(conn_az)
    ih2 = ih2_azurra(h)
    #c_ih2 = controller_azurra(ih2, 'P', 'Position')
    c_ih2 = controller_azurra(ih2, 'PID', 'Current')
    p_ih2 = controller_azurra(ih2, 'P', 'Position')
    pd_ih2 = controller_azurra(ih2, 'PD', 'Position')
    pid_ih2 = controller_azurra(ih2, 'PID', 'Position')
    f_ih2 = controller_azurra(ih2, 'PID', 'Force')
    c_ih2.control_finger(2, 200)
    #ih2.move_motor_full_speed(2,1)
    time.sleep(0.1)
    ih2.set_finger_position(0x4, 100, 'hex')
    time.sleep(2)
    ih2.open_all()
    time.sleep(2)
    ih2.set_finger_force(2, 100)
    ih2.set_finger_current(3, 100)
    ih2.get_motor_current(3)

    #ih2.set_automatic_grasp('Pistol',10, 30)
    #print(ih2.get_finger_position(2))
    time.sleep(2)
    print(ih2.get_finger_status(2))
    #print(ih2.get_finger_force(2))
    #print(ih2.get_motor_current(2))
    ih2.set_hand_posture([0,30,255,200,100], 'dec')
    #ih2.open_all()
    #time.sleep(2)
    #ih2.close_all()
    #time.sleep(2)
    #ih2.open_all()
    #time.sleep(5)
    print('---- Serial Program Starting ----')

    bts = [11000001, 11111111]
    pck = Serial_pck(bts, 'bin')
    cw = [0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48]
    pck = Serial_pck(cw, 'hex')
    tx = h.send_pck(pck.out_bytes)
    print(tx)
    time.sleep(0.2)
    cw = [0x45, 0x2]
    pck = Serial_pck(cw, 'hex')
    tx = h.send_pck(pck.out_bytes)
    print(tx)
    time.sleep(0.5)
    rx = h.get_pck(1)
    print(int.from_bytes(rx, "big"))
    conn_az.close_conn()
    print('---- Serial Program Finished ----')