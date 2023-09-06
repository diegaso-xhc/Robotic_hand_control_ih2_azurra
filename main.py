import time
from toolkits import *

baudrate = 115200 # Required baudrate for the ih2 azzurra hand
port = '/dev/ttyUSB1' # Modify here your serial port (in Windows would be a COM...  and in ubuntu /dev/...

if __name__ == '__main__':
    print('---- Stablishing Connection ----')
    conn_az = Connection_Azurre(baudrate, port) # Establish the connection with the serial device
    print('---- Connected ----')
    h = Handler(conn_az) # Takes the built serial connection and uses to send and receive byte packages
    ih2 = ih2_azurra(h)  # Declares an object of the class ih2_azurra hand

    ################## The following lines show examples of the functionalities of this repository. Uncomment the command you would like to try!
    # ih2.open_all() # Opens all degrees of freedom
    # ih2.close_all() # Closes all degrees of freedom
    # ih2.set_automatic_grasp('Pistol',20, 30) # Performs pre-programmed (from manual) grasps. See the ih2_azurra class for more references.
    # print(ih2.get_finger_position(2)) # Gets the position of finger 2
    # print(ih2.get_finger_status(2)) # Gets the status of finger 2
    # print(ih2.get_finger_force(2)) # Gets the force of finger 2
    # print(ih2.get_motor_current(2)) # Gets the current of the motor corresponding to finger 2
    # ih2.set_finger_position(0x4, 100, 'hex') # Sets finger 4 to position 100 (using hexadecimal nomenclature). Please see the function for other options
    # ih2.set_finger_force(2, 100) # Sets the force of finger 2 to 100
    # ih2.set_finger_current(3, 100) # Sets the current of finger 3 to 100
    # ih2.move_motor_full_speed(2, 1)  # Moves the motor 2 at full speed (1 for closing and 0 for opening)
    # ih2.set_hand_posture([0, 30, 255, 200, 100], 'dec')  # Sets the hand position for all degrees of freedom
    ########################################## Creating Controllers ###################################################
    p_ih2 = controller_azurra(ih2, 'P', 'Position') # Creates a proportional position controller for the hand
    pd_ih2 = controller_azurra(ih2, 'PD', 'Position') # Creates a PD position controller for the hand
    pid_ih2 = controller_azurra(ih2, 'PID', 'Position') # Creates a PID position controller for the hand
    c_ih2 = controller_azurra(ih2, 'PID', 'Current')  # Creates a PID current controller for the hand
    f_ih2 = controller_azurra(ih2, 'PID', 'Force') # Creates a PID force controller for the hand
    ###################################################################################################################
    # c_ih2.control_finger(2, 200) # Moves the finger in control mode until it reaches a position of 200
    # f_ih2.control_grasp([0,0,100,150,175]) # Moves the whole hand in force control until the position setpoints on the passed list are reached. Once achieved, the user can move the fingers and they will return to its goal position.

    ################################## Understanding the handling of bytes #############################################
    # bts = [11000001, 11111111] # Setting bits to be transmitted to the hand
    # pck = Serial_pck(bts, 'bin') # Creating a serial package with the aforementioned bits
    # cw = [0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48] # Setting an array of hexadecimal values
    # pck = Serial_pck(cw, 'hex') # Creating a serial package with the aforementioned values
    # tx = h.send_pck(pck.out_bytes) # Using the handler of the hand to send the package
    # print(tx) # Visualizing the package being transmitted
    # time.sleep(0.2) # Pause
    # cw = [0x45, 0x2] # Setting an array of hexadecimal values
    # pck = Serial_pck(cw, 'hex') # Creating a serial package with the aforementioned values
    # tx = h.send_pck(pck.out_bytes) # Using the handler of the hand to send the package
    # print(tx) # Visualizing the package being transmitted
    # time.sleep(0.5) # Pause
    # rx = h.get_pck(1) # Receiving a package from the hand
    # print(int.from_bytes(rx, "big")) # Visualizing the package being received
    ###################################################################################################################

    conn_az.close_conn() # Closes the serial connection with the hand