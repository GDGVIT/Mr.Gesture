import serial
import statistics
import pyautogui
import time
from direct_input_guy_keyboard import up,down,left,right,PressKey,ReleaseKey

def normalcursor():

    ser = serial.Serial('COM8',9600)

    while True:

        raw_dat = ser.readline().decode().strip('\r\n')
        print(raw_dat)
        data_list = raw_dat.split(',')
        x_val = data_list[0]
        y_val = data_list[1]
        z_val = data_list[2]
        moder = data_list[3]

        x = [15, 0]
        y = [15, 0]

        prevans_x = statistics.mean(x)
        prevans_y = statistics.mean(y)

        x[1] = float(x_val)
        y[1] = float(y_val)



        finalans_x = statistics.mean(x)
        finalans_y = statistics.mean(y)

        if finalans_x < prevans_x :
            pyautogui.moveRel(-20, None)
        else:
            pyautogui.moveRel(20, None)

        if finalans_y > prevans_y:
            pyautogui.moveRel(0, -20)
        else:
            pyautogui.moveRel(0, 20)

        if float(z_val) < -35.0:

            xb,yb = pyautogui.position()
            pyautogui.click(xb,yb)
            time.sleep(0.2)

        if moder == 1:
            break






def frame_check():
    pyautogui.moveTo(x=960,y=540)
    print('iam soon gonna change')
    #time.sleep(1)
    pyautogui.moveRel(20)


def gamer_mode():

    ser = serial.Serial('COM8', 9600)

    while True:
        raw_dat = ser.readline().decode().strip('\r\n')
        print(raw_dat)
        data_list = raw_dat.split(',')
        x_val = data_list[0]
        y_val = data_list[1]
        z_val = data_list[2]
        moder = data_list[3]

        if float(z_val) < -8:
            PressKey(up)
            PressKey(left)
            time.sleep(0.15)
            ReleaseKey(left)


        elif float(z_val) > 8:
            PressKey(up)
            PressKey(right)
            time.sleep(0.15)
            ReleaseKey(right)

        else:
            ReleaseKey(right)
            ReleaseKey(left)
            PressKey(up)

        if moder == 0:
            break


while True:
    normalcursor()
    gamer_mode()






