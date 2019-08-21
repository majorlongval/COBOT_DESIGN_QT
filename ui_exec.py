from PyQt5.QtWidgets import*
from PyQt5.uic import loadUi
import sys
import numpy as np
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT
                                                as NavigationToolbar)
import random


class cst_plane_Robot_Link:
    def __init__(self, Length=1, mass=1, ip=(0, 0), ang=0):
        self.Length = Length
        self.mass = mass
        self.ip = ip
        self.ang = ang
        self.fp = self.calculate_fp(self.ang, self.Length, self.ip)

    def calculate_fp(self, ang, Length, ip):
        rot_mat = np.array([[np.cos(ang), -np.sin(ang)],
                            [np.sin(ang), np.cos(ang)]])
        rel_vect = np.round(np.dot(rot_mat, np.array([Length, 0])), 5)
        fp = rel_vect + ip
        return fp

    def rotate(self, nang):
        self.ang = nang
        self.fp = self.calculate_fp(nang, self.Length, self.ip)

    def plot(self, ax, **kwargs):
        ax.plot([self.ip[0], self.fp[0]], [self.ip[1], self.fp[1]], **kwargs)
        return self.fp


class plane_COBOT:
    def __init__(self, t1=np.pi/4, t2=np.pi, t3=-np.pi/4,
                 L1=1, L2=0.2, L3=1.2, L4=0.5,
                 M1=1, M2=1, M3=1, M4=1, I1=1, I2=1,
                 I3=1, I4=1, bp=[0, 0]):

        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4
        self.M1 = M1
        self.M2 = M2
        self.M3 = M3
        self.M4 = M4
        self.I1 = I1
        self.I2 = I2
        self.I3 = I3
        self.I4 = I4
        self.bp = bp
        self.PGDp = self.PGD()
        self.Link_1 = cst_plane_Robot_Link(ip=np.array(self.bp),
                                           Length=self.L1, ang=self.t1)
        self.Link_2 = cst_plane_Robot_Link(ip=np.array(self.bp),
                                           Length=self.L2, ang=self.t2)
        self.Link_3 = cst_plane_Robot_Link(ip=self.PGDp[1],
                                           Length=-self.L3, ang=self.t2)
        self.Link_4 = cst_plane_Robot_Link(ip=self.PGDp[2],
                                           Length=self.L4, ang=self.t3)
        self.Link_5 = cst_plane_Robot_Link(ip=self.PGDp[0],
                                           Length=self.L1, ang=self.t1)

    def PGD(self):
        e1 = np.array([np.cos(self.t1), np.sin(self.t1)])
        e2 = np.array([np.cos(self.t2), np.sin(self.t2)])
        e3 = np.array([np.cos(self.t3+self.t2+self.t1),
                       np.sin(self.t3+self.t2+self.t1)])
        p1 = np.array(self.bp)+(self.L2*e2)
        p2 = p1 + (self.L1*e1)
        p3 = p2 - (self.L3*e2)
        p = p3 + (self.L4*e3)
        return [p1, p2, p3, p]

    def update(self):
        self.PGDp = self.PGD()
        self.Link_1 = cst_plane_Robot_Link(ip=np.array(self.bp),
                                           Length=self.L1, ang=self.t1)
        self.Link_2 = cst_plane_Robot_Link(ip=np.array(self.bp),
                                           Length=self.L2, ang=self.t2)
        self.Link_3 = cst_plane_Robot_Link(ip=self.PGDp[1],
                                           Length=-self.L3, ang=self.t2)
        self.Link_4 = cst_plane_Robot_Link(ip=self.PGDp[2],
                                           Length=self.L4, ang=self.t3)
        self.Link_5 = cst_plane_Robot_Link(ip=self.PGDp[0],
                                           Length=self.L1, ang=self.t1)

    def plot(self, ax, **kwargs):
        self.Link_2.plot(ax, **kwargs)
        self.Link_3.plot(ax, **kwargs)
        self.Link_1.plot(ax, **kwargs)
        self.Link_4.plot(ax, **kwargs)
        self.Link_5.plot(ax, **kwargs)

    def PGI(self, des_pose=[2,0,-np.pi/4]):
        des_pose = np.array(des_pose)



class MatplotlibWidget(QMainWindow):

    def __init__(self):

        QMainWindow.__init__(self)

        loadUi("cobot_design_ui.ui", self)

        self.setWindowTitle("PyQt5 & Matplotlib Example GUI")
        self.addToolBar(NavigationToolbar(self.MplWidget.canvas, self))
        # Setting the default values
        # Adding code to plug the stuff
        self.T1_Value_Box.editingFinished.connect(lambda: self.set_slider_pos(self.T1_Value_Box.value(), self.T1_Slider))
        self.T2_Value_Box.editingFinished.connect(lambda: self.set_slider_pos(self.T2_Value_Box.value(), self.T2_Slider))
        self.T3_Value_Box.editingFinished.connect(lambda: self.set_slider_pos(self.T3_Value_Box.value(), self.T3_Slider))
        self.T1_Slider.valueChanged['int'].connect(lambda: self.set_Value_Box_val(self.T1_Value_Box, self.T1_Slider))
        self.T2_Slider.valueChanged['int'].connect(lambda: self.set_Value_Box_val(self.T2_Value_Box, self.T2_Slider))
        self.T3_Slider.valueChanged['int'].connect(lambda: self.set_Value_Box_val(self.T3_Value_Box, self.T3_Slider))
        self.T1_Value_Box.valueChanged.connect(lambda: self.update_cobot(t1=self.T1_Value_Box.value()))
        self.T2_Value_Box.valueChanged.connect(lambda: self.update_cobot(t2=self.T2_Value_Box.value()))
        self.T3_Value_Box.valueChanged.connect(lambda: self.update_cobot(t3=self.T3_Value_Box.value()))
        self.L1_Value_Box.valueChanged.connect(lambda: self.update_cobot(L1=self.L1_Value_Box.value()))
        self.L2_Value_Box.valueChanged.connect(lambda: self.update_cobot(L2=self.L2_Value_Box.value()))
        self.L3_Value_Box.valueChanged.connect(lambda: self.update_cobot(L3=self.L3_Value_Box.value()))
        self.L4_Value_Box.valueChanged.connect(lambda: self.update_cobot(L4=self.L4_Value_Box.value()))
        self.cobot = plane_COBOT()

    def set_slider_pos(self, ang_pos, slider):
        rel_pos = ((slider.maximum()-slider.minimum())/(2*np.pi))*ang_pos
        slider.setSliderPosition(rel_pos)

    def set_Value_Box_val(self, Value_Box, slider):
        Value_Box.setValue(2*np.pi*slider.value()/(slider.maximum()-slider.minimum()))

    def change_cobot_values(self, **kwargs):
        self.cobot.__dict__.update(kwargs)
        self.cobot.update()

    def print_cobot(self):
        self.MplWidget.canvas.axes.clear()
        self.MplWidget.canvas.axes.set_xlim([-1, 2])
        self.MplWidget.canvas.axes.set_ylim([-1, 2])
        self.MplWidget.canvas.axes.set_aspect(1.0)
        self.MplWidget.canvas.axes.grid('on')
        self.cobot.plot(self.MplWidget.canvas.axes, lw=5, marker='.',
                        color='b', markersize=20, markerfacecolor='r',
                        markeredgecolor='r', alpha=0.5)
        self.MplWidget.canvas.draw()

    def update_cobot(self, **kwargs):
        self.change_cobot_values(**kwargs)
        self.print_cobot()

if __name__ == "__main__":
    app = QApplication([])
    window = MatplotlibWidget()
    window.show()
    app.exec_()
