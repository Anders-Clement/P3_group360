#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

import tkinter as tk

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import numpy as np
import sympy as sp

time = sp.symbols('time', real=True)

class Application(tk.Frame):
    def __init__(self, master=None):
        print(master)
        super().__init__(master)
        self.master = master
        self.buttons =  ["Step","Sinus"]
        self.buttonsFunction = [time,sp.sin(time)]
        self.checkBoxs = [0] * self.buttons.__len__()
        self.checkBoxsVariables = []
        self.grid()
        self.create_widgets()
        self.create_inputs()
        self.matplot()
        self.robotPos = [[],[]]
        self.robotSetPos = [0,0,0,0,0]
        self.robotCapturing = False
        self.plotUpdate = False

        rospy.Subscriber("/crustcrawler/getAngleVel", Float64MultiArray, self.trajectoryCallback)
        self.trajPub = rospy.Publisher('/crustcrawler/trajectory', Float64MultiArray)
        rospy.Timer(rospy.Duration(1.0/20.0), self.trajectoryUpdate)

    def create_widgets(self):
        self.buttonFrame = tk.Frame(self.master)
        self.buttonFrame.grid(column=0, row=0)

        for i in range(0, self.buttons.__len__()):
            var = tk.IntVar()
            self.checkBoxs[i] = tk.Checkbutton(self.buttonFrame, variable=var)
            self.checkBoxs[i]["text"] = self.buttons[i]
            self.checkBoxs[i]["command"] = self.disableButton
            self.checkBoxsVariables.append(var)
            self.checkBoxs[i].grid(row=i+8, column=0)

        self.updatePlotButton = tk.Button(self.buttonFrame, text="Update Plot", fg="black", command=self.updatePlot)
        self.updatePlotButton.grid(row=5,column=0)

        self.captureButton = tk.Button(self.buttonFrame, text="Capture Robot", fg="black", command=self.startCapture)
        self.captureButton.grid(row=6,column=0)

    def create_inputs(self):
        self.inputFrame = tk.Frame(self.buttonFrame)
        self.inputFrame.grid(column=0, row=0)

        labels = ["Time (sec)", "Freq (Hz)", "Multi (-)", "Offset (-)", "Joint (-)", "Joint0 (rad)"]
        defaults = [np.pi, 1, 1, 0, 0, 0]

        self.inputBox = []

        self.statusLabel = tk.Label(self.buttonFrame, text="Status: ")
        self.statusLabel.grid(row=1, column=0)

        for i in range(0,labels.__len__()):
            tk.Label(self.inputFrame, text=labels[i]).grid(row=i, column=0)
            self.inputBox.append(tk.Entry(self.inputFrame))
            self.inputBox[i].insert(tk.INSERT,(defaults[i]))
            self.inputBox[i].grid(row=i, column=1)
            self.inputBox[i].bind("<Enter>", self.disableButton)

    def matplot(self):
        fig = Figure(figsize=(5, 4), dpi=100)

        self.canvasSubplot = fig.add_subplot(111)

        self.canvasFrame = tk.Frame(self.master)
        self.canvasFrame.grid(column=1, row=0)

        self.canvas = FigureCanvasTkAgg(fig, self.canvasFrame)  # A tk.DrawingArea.
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()

        self.toolbar = NavigationToolbar2Tk(self.canvas, self.canvasFrame)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(anchor=tk.S)

    def updatePlotButton(self):
        self.statusLabel["text"] = "Status: Updating the plot."
        self.plotUpdate = True

    def updatePlot(self):

        self.t = np.arange(0, float(self.inputBox[0].get()), (1.0/20.0))
        self.Y = []
        self.jointSelected = int(self.inputBox[4].get())

        self.robotSetPos[0] = float(self.inputBox[5].get())

        empty = sp.symbols('empty')
        finalFunction = empty

        subs = [ 1.0 , 2*np.pi*float(self.inputBox[1].get())*time + float(self.inputBox[3].get()) ]

        if (subs.__len__() != self.buttonsFunction.__len__()):
            return

        for i in range(0, self.buttonsFunction.__len__()):
            if self.checkBoxsVariables[i].get() == 1:
                finalFunction += self.buttonsFunction[i].subs(time, subs[i]) * float(self.inputBox[2].get())

        finalFunction = finalFunction.subs(empty, 0)

        for i in range(0, self.t.__len__()):
            currentY = finalFunction.subs(time, self.t[i]).doit()
            self.Y.append( currentY )

        self.canvasSubplot.clear()
        self.canvasSubplot.grid(color='k', alpha=0.3, linestyle='-', linewidth=0.5)

        self.canvasSubplot.plot(self.t,self.Y, 'b')

        self.canvasSubplot.plot(self.robotPos[0], self.robotPos[1], 'r')

        self.canvas.draw()
        self.statusLabel["text"] = "Status: Plot Updated."
        self.enableButton()

    def trajectoryCallback(self, inputData):
        if (self.robotCapturing):
            self.robotPos[0].append(rospy.get_rostime().to_sec())
            self.robotPos[1].append(inputData.data[self.jointSelected*2])

    def trajectoryUpdate(self, event):
        if self.robotCapturing == False:
            return

        data = Float64MultiArray()
        data.data = [0]*15

        if (self.robotCapturing):
            timeNow = rospy.get_rostime().to_sec() - self.robotCaptureStartTime
            YIndex = 0

            for i in range(0, self.t.__len__()):
                if (self.t[i] > timeNow):
                    YIndex = i
                    break

            self.robotSetPos[self.jointSelected] = self.Y[YIndex]

        for i in range(0, 5):
            data.data[i*3] = self.robotSetPos[i]

        self.trajPub.publish(data)

    def startCapture(self):
        rospy.Timer(rospy.Duration(float(self.inputBox[0].get())), self.stopCapture, True)
        self.statusLabel["text"] = "Status: Capturing Robot."
        self.robotCaptureStartTime = rospy.get_rostime().to_sec()
        self.robotCapturing = True
        self.robotPos = [[],[]]

    def stopCapture(self, event):
        self.robotCapturing = False

        for i in range(0, self.robotPos[0].__len__()):
            self.robotPos[0][i] = self.robotPos[0][i] - self.robotCaptureStartTime

        self.plotUpdate = True
        self.statusLabel["text"] = "Status: Capture completed."

    def updater(self):
        if (self.plotUpdate):
            self.updatePlot()
            self.plotUpdate = False

    def disableButton(self, event=None):
        self.captureButton.config(state=tk.DISABLED)

    def enableButton(self):
        self.captureButton.config(state=tk.NORMAL)


def mainFun():
    rospy.init_node('CrustCrawler_Plotter', anonymous=True)
    root = tk.Tk()
    app = Application(master=root)

    while not rospy.is_shutdown():
        app.updater()
        app.update()

    root.destroy()

if __name__ == '__main__':
    mainFun()
