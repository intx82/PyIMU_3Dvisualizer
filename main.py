import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from figure import *
from tkinter import *
from tkinter import ttk
import time
import serial
import threading
import json

ApplicationGL = False


class PortSettings:
    Name = "COM15"
    Speed = 115200
    Timeout = 2


class IMU:
    Roll = 0
    Pitch = 0
    Yaw = 0


class KalmanFilter:

    def __init__(self, A : float, H: float, Q: float, R: float, initial_P: float, initial_x: float):
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        self.P = initial_P
        self.x = initial_x

    def output(self, inp):
        self.x = self.A * self.x
        self.P = self.A * self.P * self.A + self.Q
        K = self.P * self.H / (self.H * self.P * self.H + self.R)
        self.x = self.x + K * (inp - self.H * self.x)
        self.P = (1 - K * self.H) * self.P
        return self.x


_com_port = PortSettings()
myimu = IMU()


def RunAppliction():
    global ApplicationGL
    _com_port.Name = Port_entry.get()
    _com_port.Speed = 115200
    ApplicationGL = True
    ConfWindw.destroy()


ConfWindw = Tk()
ConfWindw.title("Configure Serial Port")
ConfWindw.configure(bg="#2E2D40")
ConfWindw.geometry('300x100')
ConfWindw.resizable(width=False, height=False)
positionRight = int(ConfWindw.winfo_screenwidth()/2 - 300/2)
positionDown = int(ConfWindw.winfo_screenheight()/2 - 100/2)
ConfWindw.geometry("+{}+{}".format(positionRight, positionDown))

Port_label = Label(text="Port:", font=("", 12), justify="right", bg="#2E2D40", fg="#FFFFFF")
Port_label.place(x=50, y=30, anchor="center")
Port_entry = Entry(width=20, bg="#37364D", fg="#FFFFFF", justify="center")
Port_entry.insert(INSERT, _com_port.Name)
Port_entry.place(x=180, y=30, anchor="center")

ok_button = Button(text="Ok", width=8, command=RunAppliction, bg="#135EF2", fg="#FFFFFF")
ok_button.place(x=150, y=75, anchor="center")


def InitPygame():
    global display
    pygame.init()
    display = (640, 480)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption('IMU visualizer   (Press Esc to exit)')


def InitGL():
    glClearColor((1.0/255*46), (1.0/255*45), (1.0/255*64), 1)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    gluPerspective(100, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)


def DrawText(textString):
    font = pygame.font.SysFont("Courier New", 25, True)
    textSurface = font.render(
        textString, True, (255, 255, 0), (46, 45, 64, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(),
                 GL_RGBA, GL_UNSIGNED_BYTE, textData)


def DrawBoard():

    glBegin(GL_QUADS)
    x = 0

    for surface in surfaces:

        for vertex in surface:
            glColor3fv((colors[x]))
            glVertex3fv(vertices[vertex])
        x += 1
    glEnd()


def DrawGL():

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    gluPerspective(90, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    glRotatef(round(myimu.Pitch, 1), 0, 0, 1)
    glRotatef(round(myimu.Roll, 1), -1, 0, 0)

    DrawText(f"Roll: { round(myimu.Roll, 1)}° Pitch: {round(myimu.Pitch, 1)}°")
    DrawBoard()
    pygame.display.flip()


def SerialConnection():
    global serial_object
    serial_object = serial.Serial(
        _com_port.Name, baudrate=_com_port.Speed, timeout=_com_port.Timeout)


def ReadData():
    kf_roll = None
    kf_pitch = None
    while True:
        serial_input = serial_object.readline()
        try:
            if len(serial_input) > 0:
                angles = json.loads(serial_input)
                if kf_roll is None:
                    kf_roll = KalmanFilter(1, 1, 0.125, 1, 0.5, angles['y'])
                else:
                    myimu.Roll = kf_roll.output(angles['y'])
                
                if kf_pitch is None:
                    kf_pitch = KalmanFilter(1, 1, 0.125, 1, 0.5, angles['x'])
                else:
                    myimu.Pitch = kf_pitch.output(angles['x'])
                print(f'roll: {myimu.Roll}, pitch: {myimu.Pitch}')

        except Exception as ex:
            print(ex.with_traceback(3))

def main():
    ConfWindw.mainloop()
    if ApplicationGL == True:
        InitPygame()
        InitGL()

        try:
            SerialConnection()
            _serial_thread = threading.Thread(target=ReadData)
            _serial_thread.daemon = True
            _serial_thread.start()
            while True:
                event = pygame.event.poll()
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    pygame.quit()
                    break

                DrawGL()
                pygame.time.wait(10)

        except:
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            DrawText("Sorry, something is wrong :c")
            pygame.display.flip()
            time.sleep(5)


if __name__ == '__main__':
    main()
