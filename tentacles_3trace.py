#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 25 10:50:29 2018

# 根据运动学方程生成一段指定时间长度end_time的轨迹，该轨迹由3段曲线组成，分为(0：time0)~(end_time-time0：end_time)
  ## 时间只有1个，另一个对称
  ## 方向盘转角速度只有一个
  ## 偏角可以有3个
  ## 加速度可以有3个

@author: taosheng
"""

import numpy as np
import matplotlib.pyplot as plt

###############
# 车辆参数
# 1.轴距
# 2.最大偏角
# 3.最大转速度
# 4.最大加速度
# 5.最大减速度
# 6.轮距
# 7.质量
class car_para:
  def __init__(self):
    self.l = 2.5 # 轴距2.5m
    self.max_alpha = 30.0*np.pi/180.0 # 最大前轮偏角
    self.max_omiga = self.max_alpha/3.0 # 最大前轮偏角转速度
    self.max_accel = 10.0 # 最大加速度
    self.max_dec = -10.0 # 最大减速度
    self.wheel = 2.0 # 轮距
    self.weight = 1500 # 质量

###############
# 车辆姿态
# 1.坐标x
# 2.坐标y
# 3.速度v
## 4.加速度a
# 5.朝向
# 6.前轮偏角
class car_pos:
  def __init__(self):
    self.x0 = 0.0 #
    self.y0 = 0.0
    self.v0 = 0.0 # 速度
    self.theta0 = 0.0 # 朝向
    self.alpha0 = 0.0 # 前轮偏角

###############
# 车辆规划
# 1.预期最大偏角
# 2.预期转角速度0
# 3.预期加速度0
# 4.规划时间序列
# 5.规划时间间隔
class car_plan:
  def __init__(self):
    self.max_alpha = 0.0
    self.omiga0 = 0.0
    self.a0 = 0.0
    self.time_list = []
    self.dt = 0.0

###############
# 地图元素
class map_element:
  def __init__(self):
    self.speed_limit = 40.0 # 速度限制 40m/s
    self.accel_r = 5.0 # 向心加速度限制 5.0m/s^2

# 计算t时间内的一段轨迹
#     坐标            速度      朝向         偏角
# pos:x0=0.0, y0=0.0, v0=0.0, theta0=0.0, alpha0=0.0,
#      最大偏角        转角速度  加速度
# plan:max_alpha=0.0, omiga0, a0, time_list, dt
class traject_paras:
  ######################
  # 计算一段轨迹
  ######################
  def build_trace(self, car, pos, plan, element):
    # 初始化变量
    x = np.zeros(plan.time_list.size)
    x[0] = pos.x0  
    y = np.zeros(plan.time_list.size)
    y[0] = pos.y0  
    v = np.zeros(plan.time_list.size)
    v[0] = pos.v0  
    theta = np.zeros(plan.time_list.size)
    theta[0] = pos.theta0  
    alpha = np.zeros(plan.time_list.size)
    alpha[0] = pos.alpha0  
    i = 0
    # 转角方向
    if plan.max_alpha>pos.alpha0:
      steer = 1
    else:
      steer = -1
    # 计算轨迹  
    for t in plan.time_list[1:]:
      # 计算向心加速度要求下的可取值前轮偏角
      if(v[i]==0):
        soft_alpha = np.pi
      else:
        if(element.accel_r*car.l/(v[i]*v[i])<1 and element.accel_r*car.l/(v[i]*v[i])>-1):
          soft_alpha = np.arcsin(element.accel_r*car.l/(v[i]*v[i]))
        else:
          soft_alpha = np.pi
      # 因为分辨率的问题（即omiga0的取值问题，并不能完美的达到max_alpha或者soft_alpha，为了保证不侧滑，角度不超）
      alpha[i+1] = alpha[i]+plan.omiga0*plan.dt*steer
      # 不同方向有不同的正负值
      if steer>0:
        if alpha[i+1]>plan.max_alpha or alpha[i+1]>soft_alpha:
  #        alpha[i+1] = alpha[i]
          alpha[i+1] = min(plan.max_alpha, soft_alpha)
      else:
        if alpha[i+1]<plan.max_alpha or alpha[i+1]<(-soft_alpha):
  #        alpha[i+1] = alpha[i]
          alpha[i+1] = max(plan.max_alpha, -soft_alpha)
      # 偏角导致航向变化
      theta[i+1] = theta[i]+v[i]*np.tan(alpha[i])/car.l*plan.dt
      # 位置变化
      x[i+1] = x[i]+v[i]*np.cos(theta[i])*plan.dt
      y[i+1] = y[i]+v[i]*np.sin(theta[i])*plan.dt
      # 速度变化
      v[i+1] = v[i]+plan.a0*plan.dt
      # 速度不能超限
      if v[i+1]>element.speed_limit:
        v[i+1] = v[i]
      elif v[i+1]<0:
        v[i+1] = 0
      i += 1
    return x,y,v,theta,alpha

  ######################
  # 计算连续三段轨迹作为一根触须
  ######################  
  def build_tentacle(self, car, pos0, plan0, plan1, plan2, element):
    pos = car_pos()
    # 第一段
    x,y,v,theta,alpha = self.build_trace(car, pos0, plan0, element)
    # 第二段
    pos.x0 = x[-1]
    pos.y0 = y[-1]
    pos.v0 = v[-1]
    pos.alpha0 = alpha[-1]
    pos.theta0 = theta[-1]
    x1,y1,v1,theta1,alpha1 = self.build_trace(car, pos, plan1, element)
    x = np.hstack((x,x1))
    y = np.hstack((y,y1))
    v = np.hstack((v,v1))
    alpha = np.hstack((alpha, alpha1))
    theta = np.hstack((theta, theta1))
    # 第三段
    pos.x0 = x[-1]
    pos.y0 = y[-1]
    pos.v0 = v[-1]
    pos.alpha0 = alpha[-1]
    pos.theta0 = theta[-1]
    x1,y1,v1,theta1,alpha1 = self.build_trace(car, pos, plan2, element)
    x = np.hstack((x,x1))
    y = np.hstack((y,y1))
    v = np.hstack((v,v1))
    alpha = np.hstack((alpha, alpha1))
    theta = np.hstack((theta, theta1))
    #            
    return x,y,v,theta,alpha

  ######################
  # 随机生成一根触须：
  ## 时间2个
  ## 方向盘转角速度3个
  ## 偏角可以有3个
  ## 加速度可以有3个
  ######################  
  def rand_build_tentacle(self, car, pos0, element, end_time):
    done = 0
    plan0 = car_plan()
    plan1 = car_plan()
    plan2 = car_plan()
      
    while(done==0):
      # 时间
      time0 = np.random.randint(2, end_time*10-2)
      time1 = np.random.randint(time0, end_time*10)
      time0 = time0/10
      time1 = time1/10
      # 转角速度
      omiga0 = np.random.rand()*car.max_omiga
      omiga1 = np.random.rand()*car.max_omiga
      omiga2 = np.random.rand()*car.max_omiga
      # 规划0
      plan0.a0 = np.random.uniform(-1.0,1.0)*car.max_accel
      plan0.omiga0 = omiga0
      plan0.max_alpha = np.random.uniform(-1.0,1.0)*car.max_alpha
      plan0.dt = 0.1
      plan0.time_list = np.arange(0, time0, plan0.dt)
#      print("plan0 time:", plan0.time_list)
      # 规划1
      plan1.a0 = np.random.uniform(-1.0,1.0)*car.max_accel
      plan1.omiga0 = omiga1
      plan1.max_alpha = np.random.uniform(-1.0,1.0)*car.max_alpha
      plan1.dt = 0.1
      plan1.time_list = np.arange(time0, time1, plan1.dt)
#      print("plan1 time:", plan1.time_list)
      # 规划2
      plan2.a0 = np.random.uniform(-1.0,1.0)*car.max_accel
      plan2.omiga0 = omiga2
      plan2.max_alpha = np.random.uniform(-1.0,1.0)*car.max_alpha
      plan2.dt = 0.1
      plan2.time_list = np.arange(time1, end_time, plan2.dt)
#      print("plan2 time:", plan2.time_list)
      
      if(len(plan0.time_list)!=0 and len(plan1.time_list)!=0 and len(plan2.time_list)!=0):
        done = 1
    
    x,y,v,theta,alpha = self.build_tentacle(car, pos0, plan0, plan1, plan2, element)
    return x, y, v, theta, alpha, plan0, plan1, plan2
    
#############################
if __name__ == '__main__':
  car = car_para()
  traject = traject_paras()
  
  # 地图参数初始化
  element = map_element()
  element.speed_limit = 30

  # 车辆初始状态
  pos0 = car_pos()
  pos0.x0 = 0.0
  pos0.y0 = 0.0
  pos0.v0 = 20.0
  pos0.alpha0 = 0.0
  pos0.theta0 = 0.0
    # 生成轨迹束
  end_time = 3
  x,y,v,theta,alpha,plan0,plan1,plan2 = traject.rand_build_tentacle(car, pos0, element, end_time)
  plt.plot(x,y)
  plt.title("tentacle")
  plt.grid("on")
