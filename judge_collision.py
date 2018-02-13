#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  9 17:47:15 2018

@author: taosheng
"""

import numpy as np

class collision_cost:
  def __init__(self):
    return
  
  def calc_collision(self, x, y, perception_list, time_range):
    collision = 0
    for i in range(time_range):
      perception = perception_list[i]
      px = int(x[i]*2)
      py = int(y[i]*2)
      bond = perception.shape
      if np.max(py)>(bond[1]-2) or np.min(y)<2:
        collision += 1
      else:
        collision += np.sum(perception[px,py])
    return collision

  """
  计算轨迹的代价
  1.最远距离：超过v*t的，都得满分10
  2.终点速度：越大越好，与限速比较，等于限速为10分
  3.前轮偏角差：最大-最小偏角差最小的，得分越高，10分
  4.体感：最大-最小加速度差最小的，得分越高，10分
  5.终点朝向：和道路朝向越吻合，得分越高，10分
  6.碰撞:有碰撞则减少得分，碰撞一次减少10分
  7.与计划轨迹的重合程度：？判断方法（终点位置）
  目前各个项目的比例因子都是1，具体多少待定
  """
  def calc_cost(self, v0, end_time, v_last, accel_diff, theta_last, alpha_diff, total_s, collision, element):
    total_cost = 0.0
    # length
    if(total_s>=v0*end_time):
      total_cost += 10.0*5
    else:
      total_cost += total_s/(v0*end_time)*10*5
    # 速度，与路段限速比较
#    total_cost += v_last%element.speed_limit*10
    # 偏角差，与2pi比，越小得分越高
    total_cost += (1-alpha_diff/(2*np.pi))*10
    # 速度差，与限速相比，越小得分越高
    total_cost += (1-accel_diff/20)*10
    # 终点朝向
    total_cost += (1-theta_last/(2*np.pi))*10 # 目前道路朝向是0度（世界坐标系），以后放到element中
    # 碰撞
    total_cost -= 20*collision
    if(total_cost<0):
      total_cost = 0
    return total_cost
  
  
