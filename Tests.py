# This is a sample Python script.

# Press Umschalt+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import serial
import time
from toolkits import *

baudrate = 115200
port = 'COM5'

if __name__ == '__main__':
    print('---- Stablishing Connection ----')
    conn_az = Connection_Azurre(baudrate, port)
    h = Handler(conn_az)

    print('---- Serial Program Starting ----')

    bts = [11000001, 11111111]
    pck = Serial_pck(bts, 'bin')
    cw = [0x48, 0x00, 0x20, 0x63, 0x40, 0x50, 0x48]
    pck = Serial_pck(cw, 'hex')
    tx = h.send_pck(pck.out_bytes)
    print(tx)
    time.sleep(0.2)
    cw = [0x45, 0x2]
    pck = Serial_pck(cw, 'hex')
    tx = h.send_pck(pck.out_bytes)
    print(tx)
    time.sleep(5)
    rx = h.get_pck(1)
    print(int.from_bytes(rx, "big"))
    cw = [0x4c]
    pck = Serial_pck(cw, 'hex')
    tx = h.send_pck(pck.out_bytes)
    conn_az.close_conn()
    print('---- Serial Program Finished ----')

