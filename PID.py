import numpy as np
import matplotlib.pyplot as plt
class PID:
    def __init__(self,Kp,Ki,Kd,Kaw,T_c,T,max,min,max_rate):
        self.Kp = Kp# proportional gain
        self.Ki = Ki#integ gain
        self.Kd = Kd #drivtive gain
        self.Kaw = Kaw #anti winduo gain
        self.T_c = T_c # time const for dric filtering
        self.T = T #time step
        self.max = max #maximum
        self.min = min # minimum
        self.max_rate = max_rate # max rate of change
        self.integral = 0 # interal term
        self. err_prev = 0 # previos err
        self.deriv_prev = 0 #previos derivatice
        self.command_sat_prev = 0 # previous sat
        self.command_prev = 0 #previous command
        self.command_sat = 0 # current saturated
        self.command = 0 # current command
    def compute(self,measurement,setpoint):
        """ Steps to excute pid controller
        Inputs: 
        measurement: current measuremnt of process
        setpoint : desired value of process var """
        #compute err 
        err = setpoint - measurement
        # update integral term with anti windup
        self.integral +=self.Ki*err*self.T+self.Kaw*(self.command_sat_prev-self.command_prev)*self.T
        # calculate filtered driv
        deriv_filt=(err-self.err_prev+self.T_c*self.deriv_prev)/(self.T+self.T_c)
        self.err_prev = err
        self.deriv_prev=deriv_filt
        # calcultae command using pod eq
        self.command = self.Kp*err+self.integral+self.Kd*deriv_filt
        #store pre command 
        self.command_prev=self.command
        #sat command
        if self.command>self.max:
            self.command = self.max
        elif self.command<sat.min:
            self.command = self.min
        else:
            self.command_sat=self.command
        #appl rate limit
        if self.command_sat>self.command_sat_prev+self.max_rate*self.T:
            self.command_sat=self.command_sat_prev+self.max_rate*self.T
        elif self.command_sat>self.command_sat_prev-self.max_rate*self.T:
            self.command_sat=self.command_sat_prev-self.max_rate*self.T
        #store prev sat command
        self.command_sat_prev=self.command_sat
        def set_target(self,setpoint):
            # set target value
            self.setpoint  = setpoint
        def printTel(self):
            print("Kp: ",self.Kp)
            print("Ki: ",self.Ki)
            print("Kd: ",self.Kd)
            print("Kaw: ",self.Kaw)
            print("T_c: ",self.T_c)
            print("T: ",self.T)
            print("max: ",self.max)
            print("min: ",self.min)
            print("max_rate: ",self.max_rate)
            
