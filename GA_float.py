#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 18:40:18 2017

@author: taosheng

浮点数方法进行遗传算法
"""

import numpy as np

"""############################################################################
purpse：轮盘赌方法随机选择1个DNA
input 
  Pf: 各DNA生存率（按照升序排列）
  prei: 不能作为被选项的DNA序号
output：
  i：被选中的DNA的序号
"""
def RatioSelect(Pf, prei):
  i=prei
  while i==prei:
    m=0
    r=np.random.random() # 随机数
    for i in range(np.size(Pf)):
      m=m+Pf[i]
      # 落在某个生存率区间上
      if r<m: 
        break
  return i

"""############################################################################
purpse：进行两个DNA的粒交换
input： 
  M1，M2：输入DNA
  alpha：交叉比例
output：
  Cm1，Cm2：交换后的DNA
"""
def CrossOver(M1, M2, alpha):
  Cm1 = np.add(np.multiply(M1, alpha), np.multiply((1-alpha),M2))
  Cm2 = np.add(np.multiply(M2, alpha), np.multiply((1-alpha),M1))
  return Cm1, Cm2

"""############################################################################
purpse：DNA的变异
input： 
  M：输入DNA
  ratiok：变异比例步长
output：
  M：变换后的DNA
"""
def Mutation(M, Mmin, Mmax, ratiok):
  r = np.random.randint(10)
  if r%2==0:
    M = np.add(M, np.multiply(ratiok, np.subtract(Mmax, M)))
  else:
    M = np.subtract(M, np.multiply(ratiok, np.subtract(M, Mmin)))
  return M

"""############################################################################
遗传算法实际：f(x)=x*sin(10*pi*x)+2, x~[-1,2],精度0.01
Bn：（2-（-1））／0.01=300<2^9
"""
if __name__=='__main__':
  Mmax = 2.0 # 取值空间最大值
  Mmin = -1.0 # 取值空间最小值
  Mn = 50 # 种群个体数
  T = 500 # 代际数
  Pc = 0.5 # 交叉概率
  Pm = 0.01 # 变异概率
  E = Mn # 结束条件
  cntE = 0
  
  ratiok = 0.1 # 变异点
  alpha = 0.9 # np.random.random() # 交叉点
  
  
  # 初始化
  G = np.random.random(Mn)*3.0-1.0 # 生成一个初始族群
  func = G*np.sin(10*np.pi*G)+2 # 计算生存率：值
  preV = np.max(func)
  #func = func-np.min(func) # 去除负数
  pf = func/sum(func) # 归一化生存率
  pfsort = np.argsort(pf) # 生存率排序的序号
  pf = np.sort(pf)
  
  for tnum in range(T):
    Gs = np.zeros(Mn) # 空子代
    for inum in range(Mn/2):
      n0 = RatioSelect(pf, -1) # 选择DNA
      n1 = RatioSelect(pf, n0) # 选择DNA
      r = np.random.random()
      if r<Pc: # 交叉
        Gs[inum*2],Gs[inum*2+1]=CrossOver(G[pfsort[n0]], G[pfsort[n1]], alpha)
      else:
        Gs[inum*2]=G[pfsort[n0]]
        Gs[inum*2+1]=G[pfsort[n1]]
    
      r = np.random.random()
      if r<Pm: #变异
        Gs[inum*2] = Mutation(Gs[inum*2], Mmin, Mmax, ratiok)
  
      r = np.random.random()
      if r<Pm: #变异
        Gs[inum*2+1] = Mutation(Gs[inum*2+1], Mmin, Mmax, ratiok)
  
    r = np.random.randint(Mn) # 随机取一个位置生成一个随机值，做为外来DNA
    Gs[r] = np.random.random()*3.0-1.0
      
    r = np.random.randint(Mn) # 随机取一个位置保留最优解
    Gs[r] = G[pfsort[-1]]
      
    G = Gs    
    func = G*np.sin(10*np.pi*G)+2 # 计算生存率：值
    if preV==np.max(func):
      cntE += 1
    else:
      cntE = 0
    preV = np.max(func)
    pf = func/sum(func) # 归一化生存率
    pfsort = np.argsort(pf) # 生存率排序的序号
    pf = np.sort(pf)
    print '代数=%d, 此时x=%f, 最大值=%f' %(tnum, G[pfsort[-1]], func[pfsort[-1]])
    if cntE == E:
      if tnum > 50:
        tnum = T
        break
            
  print 'maxfunc=%f, x=%f' %(np.max(func), G[pfsort[-1]])
    
