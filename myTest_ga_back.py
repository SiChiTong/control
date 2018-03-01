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
import GA_float as ga

#############################
if __name__ == '__main__':
  car = tt3.car_para()
  traject = tt3.traject_calc()
  print("the method is para:one omiga")
  
  end_time = 5.0

  #############################################################################
  # 障碍物时空矩阵
  # 按照0.5米一格划分地图，车道1[0:7]（3.5）,车道2[7:14]（7）,车道3[14:21]（10.5）
  perception = np.zeros([600, 21]) # 300m*（3.5m*3）
  # ob0的真实尺寸为（5，4.5），因为距离分辨率为0.5m，地图划分为0.5m一格，所以 /0.5 后转化为占用多少格
  ob0=np.ones([int(5/0.5), int(4.5/0.5)]) # 占据的格数矩阵
  ob0_v=int(0/0.5) # 速度为0
  ob0_pos=[int(30/0.5), int(0/0.5)] # 位置为
  ob0_size = np.shape(ob0) # 尺寸所对应的格数
  
  ob1=np.ones([int(5/0.5), int(5.5/0.5)])
  ob1_v=int(30/0.5)
  ob1_pos=[int(45/0.5), int(3.5/0.5)]
  ob1_size = np.shape(ob1)
  
  ob2=np.ones([int(5/0.5), int(3/0.5)])
  ob2_v=int(30/0.5)
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
    perception0[(ob0_pos[0]+itime*ob0_v/10):(ob0_pos[0]+np.shape(ob0)[0]+itime*ob0_v/10), 0:np.shape(ob0)[1]] = ob0
    perception0[ob1_pos[0]+itime*ob1_v/10:ob1_pos[0]+np.shape(ob1)[0]+itime*ob1_v/10, 5:(5+np.shape(ob1)[1])] = ob1
    perception0[ob2_pos[0]+itime*ob2_v/10:50+itime*ob2_v/10, 12:(12+np.shape(ob2)[1])] = ob2
    perception_list.append(perception0)  

  #############################################################################
  # 地图参数初始化
  element = tt3.map_element()
  element.speed_limit = 30
  dt = 0.1
  #############################################################################
  # 车辆初始状态
  pos0 = tt3.car_pos()
  pos0.x0 = 0.0 # 1.5m
  pos0.y0 = 3.0
  pos0.v0 = 20.0
  pos0.alpha0 = 0.0
  pos0.theta0 = 0.0
  
  cc = jc.collision_cost()

  

  #############################################################################
  trace_num = 80 # 种群数量
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
    """
    # 生成5条直线
    if(i>=45):
      x,y,v,theta,alpha,total_s,time_list, omiga_list, alpha_list, accel_list = \
        traject.rand_build_tentacle(car, pos0, element, end_time, 0)
    # 其他生成曲线
    else:
    """
    result, para = \
      traject.rand_build_tentacle(5, car, pos0, element, end_time, 1)
    # 计算碰撞函数  
    time_range = int(para.time_array[-1]/dt)
    colli = cc.calc_collision(result.x, result.y, perception_list, time_range)

#    if(colli==0):
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
  
  plt.figure(1)
  plt.title("tentacle")
  plt.grid("on")
  for i in range(5):
    if i==0:
      plt.plot(result_array[pfsort[-1-i]].x, result_array[pfsort[-1-i]].y, color='red')
    else:
      plt.plot(result_array[pfsort[-1-i]].x, result_array[pfsort[-1-i]].y, color='blue')
      
  plt.show()
    
  #############################################################################
  # 遗传迭代
  # 初始化子代
  child_para_array = []
  child_result_array = []
  child_cost_array = []
  
  for i in range(trace_num):
    child_para_array.append([])
    child_result_array.append([])
    child_cost_array.append([])
    
  # 生成子代
  T = 10 # 迭代次数
  Pc = 0.5 # 交叉概率
  Pm = 0.01 # 变异概率
  ratiok = 0.1 # 变异点，保留父体更多的特征
  Galpha = 0.9 # np.random.random() # 交叉点,保留父体更多的特征  
  pf = np.sort(cost_array/np.sum(cost_array)) # 归一化并排序代价，即生存率
  
  i_num = 0 # 
  for tnum in range(T):
    # 生成trace_num个有效的k子代:控制参数
    while i_num<trace_num/2:
      n0 = ga.RatioSelect(pf, -1) # 选择DNA
      n1 = ga.RatioSelect(pf, n0) # 选择DNA
      r = np.random.random()
      child_para_array[i_num*2] = para_array[pfsort[n0]]
      child_para_array[i_num*2+1] = para_array[pfsort[n1]]
      # 交叉
      if r<Pc: # 交叉
          para0,para1 = ga.CrossOver(para_array[pfsort[n0]].omiga_array, para_array[pfsort[n1]].omiga_array, Galpha)
          child_para_array[i_num*2].omiga_array = para0 # 防止
          child_para_array[i_num*2+1].omiga_array = para1         

          para0,para1 = ga.CrossOver(para_array[pfsort[n0]].accel_array, para_array[pfsort[n1]].accel_array, Galpha)
          child_para_array[i_num*2].accel_array = para0 # 防止
          child_para_array[i_num*2+1].accel_array = para1         

      # 变异
      r = np.random.random()
      if r<Pm: #变异
          child_para_array[i_num*2].omiga_array = \
              ga.Mutation(child_para_array[i_num*2].omiga_array, \
                          0, car.max_omiga, ratiok)

          child_para_array[i_num*2].accel_array = \
              ga.Mutation(child_para_array[i_num*2].accel_array, \
                          car.max_dec, car.max_accel, ratiok)
      
      r = np.random.random()
      if r<Pm: #变异
          child_para_array[i_num*2+1].omiga_array = \
              ga.Mutation(child_para_array[i_num*2+1].omiga_array, \
                          0, car.max_omiga, ratiok)

          child_para_array[i_num*2+1].accel_array = \
              ga.Mutation(child_para_array[i_num*2+1].accel_array, \
                          car.max_dec, car.max_accel, ratiok)

      # 计算子代的结果
      para_cr0 = child_para_array[i_num*2]
      result_cr0 = traject.build_tentacle(car, pos0, para_cr0, element, dt)
      time_range = int(para_cr0.time_array[-1]/dt)
      colli0 = cc.calc_collision(result_cr0.x, result_cr0.y, perception_list, time_range)      
      # 计算子代的结果
      para_cr1 = child_para_array[i_num*2+1]
      result_cr1 = traject.build_tentacle(car, pos0, para_cr1, element, dt)
      time_range = int(para_cr1.time_array[-1]/dt)
      colli1 = cc.calc_collision(result_cr1.x, result_cr1.y, perception_list, time_range)      
      # 计算子代的代价，有效子代才记录
#      if(colli0==0 and colli1==0):
      v0 = pos0.v0
      v_last = result_cr0.v[-1]
      accel_diff = np.max(child_para_array[i_num*2].accel_array)-np.min(child_para_array[i_num*2].accel_array)
      theta_last = result_cr0.theta[-1]
      alpha_diff = np.max(result_cr0.alpha)-np.min(result_cr0.alpha)      
      cost0 = cc.calc_cost(v0, end_time, v_last, accel_diff, theta_last, alpha_diff, result_cr0.s, colli, element)
      child_cost_array[i_num*2] = cost0
      child_result_array[i_num*2] = result_cr0
        
      v_last = result_cr1.v[-1]
      accel_diff = np.max(child_para_array[i_num*2+1].accel_array)-np.min(child_para_array[i_num*2+1].accel_array)
      theta_last = result_cr1.theta[-1]
      alpha_diff = np.max(result_cr1.alpha)-np.min(result_cr1.alpha)      
      cost1 = cc.calc_cost(v0, end_time, v_last, accel_diff, theta_last, alpha_diff, result_cr1.s, colli, element)
      child_cost_array[i_num*2+1] = cost1
      child_result_array[i_num*2+1] = result_cr1
      i_num += 1

    for randi in range(20): # 为了增加多样性，随机生成10个有效子代
      # 选择随机位置生成一个有效子代
      r = np.random.randint(0, trace_num)

      result, para = traject.rand_build_tentacle(5, car, pos0, element, end_time, 1)
      time_range = int(para.time_array[-1]/dt)
      colli = cc.calc_collision(result.x, result.y, perception_list,time_range)      

      v0 = pos0.v0
      v_last = result.v[-1]
      accel_diff = np.max(para.accel_array)-np.min(para.accel_array)
      theta_last = result.theta[-1]
      alpha_diff = np.max(result.alpha)-np.min(result.alpha)      
      cost = cc.calc_cost(v0, end_time, v_last, accel_diff, theta_last, alpha_diff, result.s, colli, element)
      child_para_array[r] = para
      child_cost_array[r] = cost
      child_result_array.append(result)
      randi += 1
  
    # 选择随机位置保留最佳父代
    r = np.random.randint(0, trace_num)
    child_cost_array[r] = cost_array[pfsort[-1]]
    child_para_array[r] = para_array[pfsort[-1]]
    child_result_array[r] = result_array[pfsort[-1]]

    #############################################################################
    # 更新
    for i in range(trace_num):
      para_array[i] = child_para_array[i]
      result_array[i] = child_result_array[i]
      cost_array[i] = child_cost_array[i]
  
    pfsort = np.argsort(cost_array) # 生存率排序的序号
    print('now the best pos is {}'.format(pfsort[-1]))
    print('best cost is {}'.format(cost_array[pfsort[-1]]))
    print('best pos length is {}').format(result_array[pfsort[-1]].s)
    print('best pos velocity is {}').format(result_array[pfsort[-1]].v[-1])
    pf = np.sort(cost_array/np.sum(cost_array)) # 归一化并排序代价，即生存率
    
    fig1 = plt.figure(1)
    ax1 = fig1.add_subplot(111, aspect='equal')
    rect0 = plt.Rectangle((ob0_pos[0]/2, ob0_pos[1]/2), ob0_size[0]/2, ob0_size[1]/2)
    rect1 = plt.Rectangle((ob1_pos[0]/2, ob1_pos[1]/2), ob1_size[0]/2, ob1_size[1]/2)
    rect2 = plt.Rectangle((ob2_pos[0]/2, ob2_pos[1]/2), ob2_size[0]/2, ob2_size[1]/2)
    ax1.add_patch(rect0)
    ax1.add_patch(rect1)
    ax1.add_patch(rect2)

    rect0 = plt.Rectangle(((ob0_pos[0]+itime*ob0_v/10)/2, ob0_pos[1]/2), ob0_size[0]/2, ob0_size[1]/2)
    rect1 = plt.Rectangle(((ob1_pos[0]+itime*ob1_v/10)/2, ob1_pos[1]/2), ob1_size[0]/2, ob1_size[1]/2)
    rect2 = plt.Rectangle(((ob2_pos[0]+itime*ob2_v/10)/2, ob2_pos[1]/2), ob2_size[0]/2, ob2_size[1]/2)
    ax1.add_patch(rect0)
    ax1.add_patch(rect1)
    ax1.add_patch(rect2)
    
    for i in range(trace_num):
#      print('best {}''th length is {}'.format(i, result_array[pfsort[-1-i]].s))
#      print('best {}''th cost is {}'.format(i, cost_array[pfsort[-1-i]]))
      
      if i==0:
        plt.plot(result_array[pfsort[-1-i]].x, result_array[pfsort[-1-i]].y, color='red')
      else:
        plt.plot(result_array[pfsort[-1-i]].x, result_array[pfsort[-1-i]].y, color='green')
    plt.show()
