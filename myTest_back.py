#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  9 18:17:31 2018

@author: taosheng
"""

import numpy as np
import matplotlib.pyplot as plt

import tentacles_alpha_accel as tt3
import judge_collision as jc

#############################
if __name__ == '__main__':
  car = tt3.car_para()
  traject = tt3.traject_calc()
  print("the method is ""para:const time""")
  
  end_time = 3.5 # 规划时间长度
  time_num = int(end_time*2)

  #############################################################################
  # 障碍物时空矩阵
  # 按照0.5米一格划分地图，车道1[0:7]（3.5）,车道2[7:14]（7）,车道3[14:21]（10.5）
  perception = np.zeros([600, 21]) # 300m*（3.5m*3）
  # ob0的真实尺寸为（5，4.5），因为距离分辨率为0.5m，地图划分为0.5m一格，所以 /0.5 后转化为占用多少格
  ob0=np.ones([int(5/0.5), int(4.5/0.5)]) # 占据的格数矩阵
  ob0_v=int(10/0.5) # 速度为0
  ob0_pos=[int(30/0.5), int(0/0.5)] # 位置为
  ob0_size = np.shape(ob0) # 尺寸所对应的格数
  
  ob1=np.ones([int(5/0.5), int(5.5/0.5)])
  ob1_v=int(10/0.5)
  ob1_pos=[int(25/0.5), int(3.5/0.5)]
  ob1_size = np.shape(ob1)
  
  ob2=np.ones([int(5/0.5), int(3/0.5)])
  ob2_v=int(10/0.5)
  ob2_pos=[int(20/0.5), int(7/0.5)]
  ob2_size = np.shape(ob2)
  
  perception[ob0_pos[0]:(ob0_pos[0]+ob0_size[0]), ob0_pos[1]:ob0_size[1]] = ob0
  perception[ob1_pos[0]:(ob1_pos[0]+ob1_size[0]), ob1_pos[1]:(ob1_pos[1]+ob1_size[1])] = ob1
  perception[ob2_pos[0]:(ob2_pos[0]+ob2_size[0]), ob2_pos[1]:(ob2_pos[1]+ob2_size[1])] = ob2

  fig1 = plt.figure(1)
  ax1 = fig1.add_subplot(111, aspect='equal')
  rect0 = plt.Rectangle((ob0_pos[0]/2, ob0_pos[1]/2), ob0_size[0]/2, ob0_size[1]/2)
  rect1 = plt.Rectangle((ob1_pos[0]/2, ob1_pos[1]/2), ob1_size[0]/2, ob1_size[1]/2)
  rect2 = plt.Rectangle((ob2_pos[0]/2, ob2_pos[1]/2), ob2_size[0]/2, ob2_size[1]/2)
  
  ax1.add_patch(rect0)
  ax1.add_patch(rect1)
  ax1.add_patch(rect2)
  
  perception_list=[]
  for itime in range(int(end_time*10)):
    perception0 = np.zeros([600, 21])
    perception0[ob0_pos[0]+itime*ob0_v/10:ob0_pos[0]+np.shape(ob0)[0]+itime*ob0_v/10, ob0_pos[1]:ob0_pos[1]+np.shape(ob0)[1]] = ob0
    perception0[ob1_pos[0]+itime*ob1_v/10:ob1_pos[0]+np.shape(ob1)[0]+itime*ob1_v/10, ob1_pos[1]:ob1_pos[1]+np.shape(ob1)[1]] = ob1
    perception0[ob2_pos[0]+itime*ob2_v/10:ob2_pos[0]+np.shape(ob2)[0]+itime*ob2_v/10, ob2_pos[1]:ob2_pos[1]+np.shape(ob2)[1]] = ob2
    perception_list.append(perception0)  

  rect0 = plt.Rectangle(((ob0_pos[0]+itime*ob0_v/10)/2, ob0_pos[1]/2), ob0_size[0]/2, ob0_size[1]/2)
  rect1 = plt.Rectangle(((ob1_pos[0]+itime*ob1_v/10)/2, ob1_pos[1]/2), ob1_size[0]/2, ob1_size[1]/2)
  rect2 = plt.Rectangle(((ob2_pos[0]+itime*ob2_v/10)/2, ob2_pos[1]/2), ob2_size[0]/2, ob2_size[1]/2)
  ax1.add_patch(rect0)
  ax1.add_patch(rect1)
  ax1.add_patch(rect2)

  #############################################################################
  # 地图参数初始化
  element = tt3.map_element()
  element.speed_limit = 35
  dt = 0.1
  #############################################################################
  # 车辆初始状态
  pos0 = tt3.car_pos()
  pos0.x0 = 0.0 # 1.5m
  pos0.y0 = 4.5
  pos0.v0 = 20.0
  pos0.alpha0 = 0.0
  pos0.theta0 = 0.0
  
  cc = jc.collision_cost()

  

  #############################################################################
  trace_num = 100 # 种群数量
  i = 0
  compcnt = 0 # 计算次数
  
  cost_array=[]
  # 轨迹参数
  para = tt3.traject_para()
  para_array = []
  result = tt3.traject_result()
  result_array = []
  
#  plt.figure(2)
#  plt.grid('on')
  while(i<trace_num):
    compcnt += 1
    
#    end_time = np.random.randint(2,5)
    # 生成5条直线
    if(i>=45):
      result, para = \
        traject.rand_build_tentacle(time_num, car, pos0, element, end_time, 0)
    # 其他生成曲线
    else:
      result, para = \
        traject.rand_build_tentacle(time_num, car, pos0, element, end_time, 1)
    # 计算碰撞函数  
    time_range = int(para.time_array[-1]/dt)
    colli = cc.calc_collision(result.x, result.y, perception_list, time_range)

    if(colli==0):
      i += 1

      v0 = pos0.v0
      v_last = result.v[-1]
      accel_diff = np.max(para.accel_array)-np.min(para.accel_array)
      theta_last = result.theta[-1]
      alpha_diff = np.max(result.alpha)-np.min(result.alpha)      
      cost = cc.calc_cost(v0, end_time, v_last, accel_diff, theta_last, alpha_diff, result.s, colli, element)
      
      cost_array.append(cost)
      # 控制参数
      para_array.append(para)      
      # 结果序列
      result_array.append(result)
      
      plt.plot(result.x, result.y, color='green', linestyle='--')
    
  #############################################################################
  pfsort = np.argsort(cost_array) # 生存率排序的序号
  max_cost = np.max(cost_array)
  pf = np.sort(cost_array/np.sum(cost_array))
  
  plt.figure(1)
  plt.title("tentacle")
  plt.grid("on")
  for i in range(5):
    if i==0:
      plt.plot(result_array[pfsort[-1-i]].x, result_array[pfsort[-1-i]].y, color='red')
    else:
      plt.plot(result_array[pfsort[-1-i]].x, result_array[pfsort[-1-i]].y, color='blue')
      
    print('total_s=%3.2f / cost:%1.2f / v:%2.2f' %(result_array[pfsort[-1-i]].s, cost_array[pfsort[-1-i]], result_array[pfsort[-1-i]].v[-1]))

  plt.show()
  print('get {} in {} and the planning time is {}'.format(trace_num, compcnt, para_array[pfsort[-1]].time_array[-2]))
  