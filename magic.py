#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 22 10:20:28 2017

@author: taosheng
"""

import numpy as np
import matplotlib.pyplot as plt

####
# Pacejka‘98 轮胎模型认为轮胎在垂直、侧向方向上是线性的，阻尼为常量
# 这在常见侧向加速度<0.4g，侧偏角<5度的情况下对常规轮胎具有较高的拟合精度
# 这个算式里面没有考虑地面摩擦系数的影响
###

Fz = 5 # 垂直载荷
# input : slip ratio 滑移率
s = range(-20,20,1)
# longitudinal coefficients 纵向系数
b = [2.37272, -9.46, 1490, 130, 276, 0.0886, 0.00402, -0.0615, 1.2, 0.0299, -0.176]

# parameters
Cx = b[0] # 曲线因子
Dx = b[1]*Fz*Fz + b[2]*Fz # 曲线巅因子

BCDx = (b[3]*Fz*Fz + b[4]*Fz)*np.exp(-b[5]*Fz) # 纵向力零点处的纵向刚度

print ("Cx=%f, Dx=%f, BCDx=%f") %(Cx, Dx, BCDx)

Bx = BCDx/(Cx * Dx) # 刚度因子

Shx = b[9] * Fz + b[10] # 曲线的水平方向漂移

Fx = np.zeros(len(s))
for i in range(len(s)):
  kx = s[i]+Shx # 输入变量X1
  
  Svx = 0 # 曲线的垂直方向漂移
  
  Ex = b[6]*Fz*Fz + b[7]*Fz + b[8]
  
  Fx[i] = Dx * np.sin(Cx * np.arctan(Bx * kx - Ex*(Bx*kx - np.arctan(Bx*kx)))) + Svx

plt.plot(s, Fx)
