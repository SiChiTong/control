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

"""
# longitudinal force
"""
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
Fx = np.zeros(len(s)) # 

for i in range(len(s)):
  kx = s[i]+Shx # 输入变量X1
  Svx = 0 # 曲线的垂直方向漂移
  Ex = b[6]*Fz*Fz + b[7]*Fz + b[8] # 曲綫的曲率因子 
  Fx[i] = Dx * np.sin(Cx * np.arctan(Bx * kx - Ex*(Bx*kx - np.arctan(Bx*kx)))) + Svx # 纵向力

plt.plot(s, Fx)
plt.title("longitudinal force")
plt.grid(axis='both')
plt.xlabel("slip ratio")
plt.ylabel("force")

"""
# Lateral Force
"""
# input：横向侧偏角 alpha
alpha = range(-8, 8, 1)
r = 0 # 外倾角
# lateral coefficents
a = [1.65, -34, 1250, 3036, 12.8, 0.00501, -0.02103, 0.77394, 0.0022890, 0.013442,
     0.003709, 19.1656, 1.21356, 6.26206]
# parameters
Cy = a[0] # 曲线形状因子
Dy = a[1]*Fz*Fz + a[2]*Fz # 曲线巅因子
BCDy = a[3]*np.sin(2*np.arctan(Fz/a[4]))*(1-a[5]*abs(r)) # 侧向力零点处的侧向刚度
By = BCDy/(Cy*Dy) # 刚度因子
Shy = a[9]*Fz + a[10] + a[8]*r # 曲线的水平方向漂移
ky = np.add(alpha, Shy);
Svy = a[11]*Fz*r + a[12]*Fz + a[13]
Ey = a[6]*Fz*Fz + a[7]

Fy0 = np.zeros(len(alpha))
for i in range(len(alpha)):
    Fy0[i] = Dy * np.sin(Cy * np.arctan(Cy * np.arctan(By * ky[i] - Ey*(By*ky[i]
    -np.arctan(By*ky[i]))))) + Svy

plt.figure(2)
plt.plot(alpha, Fy0)
plt.title("Lateral force")
plt.grid(axis='both')
plt.xlabel("slip angle")
plt.ylabel("force")

"""
# aligning Torque
"""
# aligning coefficents
c = [2.34000, 1.4950, 6.416654, -3.57403, -0.087737, 0.098410, 0.0027699, -0.0001151,
     0.1000, -1.33329, 0.025501, -0.02357, 0.03027, -0.0647, 0.0211329, 0.89469,
     -0.099443, -3.336941]
# parameter
Cz = c[0]
Dz = c[1]*Fz*Fz + c[2]*Fz
BCDz = (c[3]*Fz*Fz + c[4]*Fz)*(1-c[5]*np.abs(r))*np.exp(-c[5]*Fz)
Bz = BCDz/(Cz*Dz)
Shz = c[11]*r + c[12]*Fz+c[13]
kz = np.add(alpha, Shz)
Svz = r*(c[14]*Fz*Fz+c[15]*Fz)+c[16]*Fz+c[17]
Ez = (c[7]*Fz*Fz + c[8]*Fz + c[9])*(1 - c[10]*np.abs(r))

Mz0 = np.zeros(len(alpha))
for i in range(len(alpha)):
    Mz0[i] = Dz*np.sin(Cz*np.arctan(Bz*kz[i] - Ez*(Bz*kz[i]-np.arctan(Bz*kz[i])))) + Svz

plt.figure(3)
plt.plot(alpha, Mz0)
plt.title("alignning Torque")
plt.grid(axis='both')
plt.xlabel("slip angle")
plt.ylabel("Torque")


