#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  9 18:17:31 2018

@author: taosheng
"""

import numpy as np
import matplotlib.pyplot as plt

import tentacles_3trace as tt3
import judge_collision as jc

#############################
if __name__ == '__main__':
  car = tt3.car_para()
  traject = tt3.traject_paras()
  
  #
  perception = np.zeros([90, 10]) # 90m*（4m*3）
  perception[25:30, 7:10] = np.ones([5,3])
  perception[55:60, 3:6] = np.ones([5,3])
  perception[15:20, 0:3] = np.ones([5,3])
  result = jc.judge_collision()
  
  # 地图参数初始化
  element = tt3.map_element()
  element.speed_limit = 30

  # 车辆初始状态
  pos0 = tt3.car_pos()
  pos0.x0 = 0.0
  pos0.y0 = 0.0
  pos0.v0 = 0.0
  pos0.alpha0 = 0.0
  pos0.theta0 = 0.0
  
  # 生成轨迹束
  end_time = 3
  x = np.zeros([30,100])
  y = np.zeros([30,100])
  v = np.zeros([30,100])
  theta = np.zeros([30,100])
  alpha = np.zeros([30,100])
  plan = tt3.car_plan()
  plan0_list=[]
  plan1_list=[]
  plan2_list=[]
  cost=np.zeros(100)

  # 生成一束
  plan.omiga0 = 0
  plan.max_alpha = pos0.alpha0
  plan.dt = 0.1
  plan.time_list = np.arange(0, end_time, plan.dt)
  j=0
  while(j==0):
    for i in range(100):
      if i<5:
        plan.a0 = np.random.uniform(-1.0,1.0)*car.max_accel
        x[:,i],y[:,i],v[:,i],theta[:,i],alpha[:,i] = traject.build_trace(car, pos0, plan, element)
        plan0,plan1,plan2 = plan,plan,plan
      else:
        x[:,i],y[:,i],v[:,i],theta[:,i],alpha[:,i],plan0,plan1,plan2 = traject.rand_build_tentacle(car, pos0, element, end_time)
      plan0_list.append(plan0)
      plan1_list.append(plan1)
      plan2_list.append(plan2)
      if(result.judge_result(x[:,i],y[:,i],perception)==0):
        j=1
        cost[i]=0
        plt.plot(x[:,i], y[:,i])
  
  