#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 21 11:14:37 2017

@author: taosheng
"""

import numpy as np
import matplotlib.pyplot as plt

# v: velocity of rear wheel
# delta: front wheel angle
# Phi: Path angle
# l: distance between front-rear wheel 
def model_motion_delta(v, delta, Phi, l):
  M = [np.cos(Phi), np.sin(Phi), np.divide(np.tan(delta),l)]
  status = np.multiply(M, v) # dX, dY, dPhi
  return status

def model_dynamic():
  s = np.abs(np.subtract(np.multiply(v, omi), 1))
  pass

x = []
y = []
Phi = []
x.append(0)
y.append(0)
Phi.append(0)

plt.figure(0)
plt.plot(x, y)
plt.figure(1)
plt.plot(Phi)
