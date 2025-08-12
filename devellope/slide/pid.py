# -*-coding:utf-8-*-
import time
import math
import csv
from collections import deque

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.derivative_history = deque(maxlen=5)

    def compute(self, target, current):
        now = time.time()
        dt = now - self.last_time

        if dt <= 0:
            dt = 1e-6

        error = target - current
        self.integral += error * dt
        raw_derivative = (error - self.prev_error) / dt
        self.derivative_history.append(raw_derivative)
        
        sorted_history = sorted(list(self.derivative_history))
        median_derivative = sorted_history[len(sorted_history) // 2]
        
        derivative = median_derivative
        
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * derivative
        output = p + i + d
        
        self.prev_error = error
        self.last_time = now
        return output, p, i, d, error, now

    def reset(self):
        """รีเซ็ต PID controller"""
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.derivative_history.clear()

class TurnPID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def compute(self, target, current):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0: 
            dt = 1e-6
            
        error = target - current
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * derivative
        output = p_term + i_term + d_term
        
        self.prev_error = error
        self.last_time = now
        
        return output, error, p_term, i_term, d_term

    def reset(self):
        """รีเซ็ต Turn PID controller"""
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()