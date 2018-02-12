#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  9 18:17:31 2018

@author: taosheng
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as ptch

import tentacles_3trace_omiga as tt3
import judge_collision as jc
import GA_float as ga

#############################
if __name__ == '__main__':
  car = tt3.car_para()
  traject = tt3.traject_paras()
  print("the method is para:one omiga")
  
  #############################################################################
  # 障碍物时空矩阵
  # 按照0.5米一格划分地图，车道1[0:7],车道2[7:14],车道3[14:21]
  perception = np.zeros([600, 21]) # 300m*（3.5m*3）
  perception[50:60, 0:9] = np.ones([10,9])
  perception[90:100, 5:16] = np.ones([10,11])
  perception[40:50, 12:21] = np.ones([10,9])
  result = jc.collision_cost()
  fig1 = plt.figure(1)
  ax1 = fig1.add_subplot(111, aspect='equal')
  ax1.add_patch(ptch.Rectangle((25,0), 5, 3.5))
  ax1.add_patch(ptch.Rectangle((50,3.5), 5, 3.5))
  ax1.add_patch(ptch.Rectangle((20,7), 5, 3.5))
  
  #############################################################################
  # 地图参数初始化
  element = tt3.map_element()
  element.speed_limit = 30
  dt = 0.1
  #############################################################################
  # 车辆初始状态
  pos0 = tt3.car_pos()
  pos0.x0 = 0.0 # 1.5m
  pos0.y0 = 2
  pos0.v0 = 20.0
  pos0.alpha0 = 0.0
  pos0.theta0 = 0.0
  
  end_time = 3.0
  cc = jc.collision_cost()

  #############################################################################
  trace_num = 50 # 种群数量
  i = 0
  compcnt = 0 # 计算次数
  
  cost_list=[]
  time_list=[]
  omiga_list=[]
  alpha_list=[]
  accel_list=[]
  x_list=[]
  y_list=[]
  
  plt.figure(2)
  plt.grid('on')
  while(i<trace_num):
    compcnt += 1
    """
    # 生成5条直线
    if(i>=45):
      x,y,v,theta,alpha,total_s,time_list, omiga_list, alpha_list, accel_list = \
        traject.rand_build_tentacle(car, pos0, element, end_time, 0)
    # 其他生成曲线
    else:
    """
    x,y,v,theta,alpha,total_s,time, omiga, accel = \
      traject.rand_build_tentacle(car, pos0, element, end_time, 1)
    # 计算碰撞函数  
    colli = cc.calc_collision(x,y,perception)
    if(colli==0):
      i += 1
#      plt.plot(x,y)
    
      v0 = pos0.v0
      v_last = v[-1]
      v_diff = np.max(v)-np.min(v)
      theta_last = theta[-1]
      alpha_diff = np.max(alpha)-np.min(alpha)      
      cost = cc.calc_cost(v0, end_time, v_last, v_diff, theta_last, alpha_diff, total_s, colli, element)
      
      cost_list.append(cost)
      # 父代
      time_list.append(time)
      omiga_list.append(omiga) # 因为omiga实际只有1个
      alpha_list.append(alpha)
      accel_list.append(accel)
      
      x_list.append(x)
      y_list.append(y)
      plt.plot(x, y)
  
  linePos=np.where(cost_list==np.max(cost_list))  
  if(np.size(linePos)>1):
    line = linePos[0][0]
  else:
    line = linePos[0]
  plt.figure(1)
  plt.plot(x_list[line[0]], y_list[line[0]],'red')
  plt.title("tentacle")
  plt.grid("on")
  plt.show()
  print("get 50 in %d, and cost is %f, length is %f" \
        %(compcnt, cost_list[line[0]], x_list[line[0]][-1]))
  
  #############################################################################
  # 针对曲线做遗传迭代
  # 遗传参数
  preV = np.max(cost)
  T = 2 # 代际数
  Pc = 0.5 # 交叉概率
  Pm = 0.01 # 变异概率
  E = trace_num # 结束条件
  cntE = 0
  
  ratiok = 0.1 # 变异点
  Galpha = 0.9 # np.random.random() # 交叉点
  
  pf = cost_list/np.sum(cost_list) # 归一化生存率
  pfsort = np.argsort(pf) # 生存率排序的序号
  pf = np.sort(pf) # 代价值排序
  
  # 开始迭代
  plt.figure(1)
  for tnum in range(T):
    # 空子代
    time_cld_list=[]
    omiga_cld_list=[]
    alpha_cld_list=[]
    accel_cld_list=[]
    cost_cld_list=[]
    
    for inum in range(trace_num/2):
      n0 = ga.RatioSelect(pf, -1) # 选择DNA
      n1 = ga.RatioSelect(pf, n0) # 选择DNA
      r = np.random.random()
      
      # 初始化
      time_cld_list.append(time_list[pfsort[n0]])
      time_cld_list.append(time_list[pfsort[n1]])
      omiga_cld_list.append(omiga_list[pfsort[n0]])
      omiga_cld_list.append(omiga_list[pfsort[n1]])
      alpha_cld_list.append(alpha_list[pfsort[n0]])
      alpha_cld_list.append(alpha_list[pfsort[n1]])
      accel_cld_list.append(alpha_list[pfsort[n0]])
      accel_cld_list.append(alpha_list[pfsort[n1]])
      if r<Pc: # 交叉
        # 选择交叉的参数
        print('----------------Cross----------------')
        para_num = np.random.randint(0,4) # 共有4个参数
        if(para_num==0): # time_list
          for timei in range(3):
            para0,para1 = ga.CrossOver(time_list[n0][timei], time_list[n1][timei], Galpha)
            time_cld_list[inum*2][timei] = para0
            time_cld_list[inum*2+1][timei] = para1
        elif(para_num==1): # omiga
          para0,para1 = ga.CrossOver(omiga_list[n0][0], omiga_list[n1][0], Galpha)
          for omigai in range(3):
            omiga_cld_list[inum*2][omigai] = para0
            omiga_cld_list[inum*2+1][omigai] = para1
        elif(para_num==2): # alpha
          for alphai in range(3):
            para0,para1 = ga.CrossOver(alpha_list[n0][alphai], alpha_list[n1][alphai], Galpha)
            alpha_cld_list[inum*2][alphai] = para0
            alpha_cld_list[inum*2+1][alphai] = para1          
        elif(para_num==3): # accel
          for acceli in range(3):
            para0,para1 = ga.CrossOver(accel_list[n0][acceli], accel_list[n1][acceli], Galpha)
            accel_cld_list[inum*2][acceli] = para0
            accel_cld_list[inum*2+1][acceli] = para1                   

      r = np.random.random()
      if r<Pm: #变异
        print('----------------Evalu----------------')
        # 选择变异的参数
        para_num = np.random.randint(0,4) # 共有4个参数
        if(para_num==0): # time_list
          time_cld_list[inum*2][0] = ga.Mutation(time_cld_list[inum*2][0], 0.2, end_time-0.3, Galpha)
          time_cld_list[inum*2][1] = ga.Mutation(time_cld_list[inum*2][1], time_cld_list[inum*2][0], end_time-0.1, Galpha)
        elif(para_num==1): # omiga
          omiga_cld_list[inum*2][0] = ga.Mutation(omiga_cld_list[inum*2][0], 0, car.max_omiga, Galpha)
          omiga_cld_list[inum*2][1] = omiga_cld_list[inum*2][1]
          omiga_cld_list[inum*2][2] = omiga_cld_list[inum*2][2]
        elif(para_num==2): # alpha
          for alphai in range(3):
            alpha_cld_list[inum*2][alphai] = ga.Mutation(alpha_cld_list[inum*2][acceli], -car.max_alpha, car.max_alpha, Galpha)
        elif(para_num==3): # accel
          for acceli in range(3):
            accel_cld_list[inum*2][acceli] = ga.Mutation(accel_cld_list[inum*2][acceli], car.max_dec, car.max_accel, Galpha)

      r = np.random.random()
      if r<Pm: #变异
        print('----------------Evalu----------------')
        # 选择变异的参数
        para_num = np.random.randint(0,4) # 共有4个参数
        if(para_num==0): # time_list
          time_cld_list[inum*2+1][0] = ga.Mutation(time_cld_list[inum*2+1][0], 0.2, end_time-0.3, Galpha)
          time_cld_list[inum*2+1][1] = ga.Mutation(time_cld_list[inum*2+1][1], time_cld_list[inum*2+1][0], end_time-0.1, Galpha)
        elif(para_num==1): # omiga
          omiga_cld_list[inum*2+1][0] = ga.Mutation(omiga_cld_list[inum*2+1][0], 0, car.max_omiga, Galpha)
          omiga_cld_list[inum*2+1][1] = omiga_cld_list[inum*2+1][0]
          omiga_cld_list[inum*2+1][2] = omiga_cld_list[inum*2+1][0]
        elif(para_num==2): # alpha
          for alphai in range(3):
            alpha_cld_list[inum*2+1][alphai] = ga.Mutation(alpha_cld_list[inum*2+1][alphai], -car.max_alpha, car.max_alpha, Galpha)
        elif(para_num==3): # accel
          for acceli in range(3):
            accel_cld_list[inum*2+1][acceli] = ga.Mutation(accel_cld_list[inum*2+1][acceli], car.max_dec, car.max_accel, Galpha)

      # 计算inum轨迹
      x,y,v,theta,alpha,total_s \
        = traject.build_tentacle(car, pos0, time_cld_list[inum*2], omiga_cld_list[inum*2], alpha_cld_list[inum*2], accel_cld_list[inum*2], element, dt)
      x_list[inum*2] = x
      y_list[inum*2] = y
      colli = cc.calc_collision(x,y,perception)
      v0 = pos0.v0
      v_last = v[-1]
      v_diff = np.max(v)-np.min(v)
      theta_last = theta[-1]
      alpha_diff = np.max(alpha)-np.min(alpha)      
      cost = cc.calc_cost(v0, end_time, v_last, v_diff, theta_last, alpha_diff, total_s, colli, element)
      cost_cld_list.append(cost)
      # 计算inum+1轨迹
      x,y,v,theta,alpha,total_s \
        = traject.build_tentacle(car, pos0, time_cld_list[inum*2], omiga_cld_list[inum*2], alpha_cld_list[inum*2], accel_cld_list[inum*2], element, dt)
      x_list[inum*2] = x
      y_list[inum*2] = y
      colli = cc.calc_collision(x,y,perception)
      v0 = pos0.v0
      v_last = v[-1]
      v_diff = np.max(v)-np.min(v)
      theta_last = theta[-1]
      alpha_diff = np.max(alpha)-np.min(alpha)      
      cost = cc.calc_cost(v0, end_time, v_last, v_diff, theta_last, alpha_diff, total_s, colli, element)
      cost_cld_list.append(cost)
      
      plt.plot(x,y)

    # 随机取一个位置生成一个随机值，做为外来DNA  
    print('----------------random----------------')
    r = np.random.randint(trace_num) 
    x,y,v,theta,alpha_cld_list[r],total_s,time_cld_list[r], omiga_cld_list[r], accel_cld_list[r]\
        = traject.rand_build_tentacle(car, pos0, element, end_time, 1)    
    colli = cc.calc_collision(x,y,perception)
    v0 = pos0.v0
    v_last = v[-1]
    v_diff = np.max(v)-np.min(v)
    theta_last = theta[-1]
    alpha_diff = np.max(alpha)-np.min(alpha)      
    cost = cc.calc_cost(v0, end_time, v_last, v_diff, theta_last, alpha_diff, total_s, colli, element)
    cost_cld_list[r] = cost
    print('?????????rand position is %d' %(r))
    # 随机取一个位置保留最优解
    print('----------------best----------------')
    r = np.random.randint(trace_num) 
    time_cld_list[r] = time_list[pfsort[-1]]
    omiga_cld_list[r] = omiga_list[pfsort[-1]]
    alpha_cld_list[r] = alpha_list[pfsort[-1]]
    accel_cld_list[r] = accel_list[pfsort[-1]]
    cost_cld_list[r] = cost_list[pfsort[-1]]
    
    # 更新父代
    time_list = time_cld_list
    omiga_list = omiga_cld_list
    alpha_list = alpha_cld_list
    accel_list = accel_cld_list 
    cost_list = cost_cld_list
    
    pf = cost_list/np.sum(cost_list) # 归一化生存率
    pfsort = np.argsort(pf) # 生存率排序的序号
    pf = np.sort(pf) # 代价值排序
