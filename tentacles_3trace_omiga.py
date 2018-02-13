#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 25 10:50:29 2018

# 根据运动学方程生成一段指定时间长度end_time的轨迹，该轨迹由n段曲线组成，分为(0：time0)（time0:time1）(time1：end_time)
  ## 时间有2个
  ## 方向盘转角速度只有一个
  ## 偏角可以有3个
  ## 加速度可以有3个

  # 返回值
  ## 1.轨迹（位置、速度、朝向、前轮偏角）
  ## 2.计划参数（时间、偏角、加速度、转角速度）
  ## 3.总距离
  ##-- 4.最大、最小速度差
  ##-- 5.最大、最小前轮偏角差
  ##-- 6.最大、最小朝向差
  ##-- 7.终点处的朝向
  ##-- 8.终点处的速度
@author: taosheng
"""

import numpy as np
import matplotlib.pyplot as plt

###############
# 车辆参数
# 1.轴距
# 2.最大偏角
# 3.最大转角速度
# 4.最大加速度
# 5.最大减速度
# 6.轮距
# 7.质量
class car_para:
  def __init__(self):
    self.l = 2.5 # 轴距2.5m
    self.max_alpha = 30.0*np.pi/180.0 # 最大前轮偏角
    self.max_omiga = self.max_alpha/3.0 # 最大前轮偏角转速度
    self.max_accel = 8.0 # 最大加速度
    self.max_dec = -10.0 # 最大减速度
    self.wheel = 2.0 # 轮距
    self.weight = 1500 # 质量
  def updata(self):
    self.para = {'l':self.l, \
                 'max_alpha':self.max_alpha, \
                 'max_omiga':self.max_omiga, \
                 'max_accel':self.max_accel, \
                 'max_dec':self.max_dec, \
                 'wheel':self.wheel, \
                 'weight':self.weight}
    self.para_list = ['l', 'max_alpha', 'max_omiga', 'max_accel', 'max_dec', 'wheel', 'weight']

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
# 车辆单段trace的规划
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
    self.time0 = 0.0
    self.time1 = 0.0
    self.dt = 0.0
    self.time_list = []

###############
# 整个轨迹的计算结果序列
# x,y,v,theta,alpha,s
class traject_result:
  def __init__(self):
    self.x=[]
    self.y=[]
    self.v=[]
    self.alpha=[]
    self.theta=[]
    self.s=[]

###############
# 整个轨迹的参数
# time/accel/alpha/omiga
class traject_para:
  def __init__(self):
    self.time_array=[]
    self.accel_array=[]
    self.alpha_array=[]
    self.omiga_array=[]

###############
# 地图元素
class map_element:
  def __init__(self):
    self.speed_limit = 40.0 # 速度限制 40m/s
    self.accel_r = 10.0 # 向心加速度限制 10.0m/s^2

# 计算t时间内的一段轨迹
#     坐标            速度      朝向         偏角
# pos:x0=0.0, y0=0.0, v0=0.0, theta0=0.0, alpha0=0.0,
#      最大偏角        转角速度  加速度
# plan:max_alpha=0.0, omiga0, a0, time_list, dt
class traject_calc:
  ######################
  # 计算一段轨迹
  ######################
  def build_trace(self, car, pos, plan, element):
    # 初始化变量
    time_len = len(plan.time_list)
    x = np.zeros(time_len)
    y = np.zeros(time_len)
    v = np.zeros(time_len)
    theta = np.zeros(time_len)
    alpha = np.zeros(time_len)
    x[0] = pos.x0  
    y[0] = pos.y0  
    v[0] = pos.v0  
    theta[0] = pos.theta0  
    alpha[0] = pos.alpha0  

    # 转角方向
    if plan.max_alpha>pos.alpha0:
      steer = 1
    else:
      steer = -1

    # 计算轨迹  
    result = traject_result()
    total_s = 0
    if(time_len<=1):
      total_s=0
      result.x = x
      result.y = y
      result.v = v
      result.alpha = alpha
      result.theta = theta
      result.s = total_s
      return result
    i = 0
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
#      total_s += v[i]*plan.dt
      total_s += x[i+1]-x[i] # 最有效的应该是计算道路路径长度，而不是车辆行程长度，这里简单用x代替
      i += 1
    # 输出结果
    result.x = x
    result.y = y
    result.v = v
    result.alpha = alpha
    result.theta = theta
    result.s = total_s
    return result

  ######################
  # 计算连续三段轨迹作为一根触须
  ######################  
  def build_tentacle(self, car, pos0, traj_para, element, dt):
    pos = car_pos()
    plan = car_plan()
    x=[]
    y=[]
    v=[]
    theta=[]
    alpha=[]
    total_s=0
    
    x.append(pos0.x0)
    y.append(pos0.y0)
    v.append(pos0.v0)
    alpha.append(pos0.alpha0)
    theta.append(pos0.theta0)
    
    for i in range(len(traj_para.time_array)):
      # pos updata
      pos.x0 = x[-1]
      pos.y0 = y[-1]
      pos.v0 = v[-1]
      pos.alpha0 = alpha[-1]
      pos.theta0 = theta[-1]
      # plan updata
      if i==0:
        plan.time_list = np.arange(0,traj_para.time_array[0],dt)
      else:
        plan.time_list = np.arange(traj_para.time_array[i-1],traj_para.time_array[i],dt)
      plan.omiga0 = traj_para.omiga_array[i]
      plan.max_alpha = traj_para.alpha_array[i]
      plan.a0 = traj_para.accel_array[i]
      plan.dt = dt
      # trace
      result = self.build_trace(car, pos, plan, element)
      # combine
      x = np.hstack((x,result.x))
      y = np.hstack((y,result.y))
      v = np.hstack((v,result.v))
      alpha = np.hstack((alpha, result.alpha))
      theta = np.hstack((theta, result.theta))
      total_s += result.s
    result.x = x
    result.y = y
    result.v = v
    result.alpha = alpha
    result.theta = theta
    result.s = total_s
    return result

  ######################
  # 随机生成一根触须：
  ## 时间2个
  ## 方向盘转角速度3个
  ## 偏角可以有3个
  ## 加速度可以有3个
  ## cureve决定是否为直线
  ######################  
  def rand_build_tentacle(self, trace_num, car, pos0, element, end_time, curve):
    dt=0.1 # 100ms
    # 初始化
    time_list = np.zeros(trace_num)
    omiga_list = np.zeros(trace_num)
    alpha_list = np.zeros(trace_num)
    accel_list = np.zeros(trace_num)
    # 计算轨迹
    for i in range(trace_num):
      # 0.1s周期，所以时间分辨率只需要0.1s
      if i==0:
        time_list[i] = np.random.randint(1, end_time*10-trace_num)
      elif i==(trace_num-1):
        time_list[i] = end_time*10
      else:
        time_list[i] = np.random.randint(time_list[i-1]+1, end_time*10-trace_num+i)
      # 将转角速度分辨率设为1/100 rad/s
      omiga_list[i] = np.random.randint(0,100)*car.max_omiga/100.0
      # 将偏角分辨率设为1/100 rad
      alpha_list[i] = np.random.randint(-100,100)*car.max_alpha/100.0
      # 将加速度分辨率设为？
      accel_list[i] = np.random.uniform(car.max_dec, car.max_accel)
    # 0.1s分度
    time_list = time_list/10
    # 当不为曲线时，方向盘转角速度为0
    omiga_list = omiga_list*curve

    para = traject_para()
    para.accel_array = accel_list
    para.alpha_array = alpha_list
    para.omiga_array = omiga_list
    para.time_array = time_list
    
    result = self.build_tentacle(car, pos0, para, element, dt)
    # 返回    
    return result, para
    
#############################
if __name__ == '__main__':
  car = car_para()
  traject = traject_calc()
  
  # 地图参数初始化
  element = map_element()
  element.speed_limit = 30

  # 车辆初始状态
  pos0 = car_pos()
  pos0.x0 = 0.0
  pos0.y0 = 0.0
  pos0.v0 = 10.0
  pos0.alpha0 = 0.0
  pos0.theta0 = 0.0
    # 生成轨迹束
  end_time = 3
  result, para = \
    traject.rand_build_tentacle(3,car, pos0, element, end_time,1)
  plt.plot(result.x, result.y)
  plt.title("tentacle")
  plt.grid("on")
  print("total length is %f" %(result.s)) 
  print('time_list={}'.format(para.time_array))
  print('omiga_list={}'.format(para.omiga_array))
  print('alpha_list={}'.format(para.alpha_array))
  print('accel_list={}'.format(para.accel_array))

  #############################
  # 