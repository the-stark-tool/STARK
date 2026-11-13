# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 17:53:46 2026

@author: 20235409
"""

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from statistics import mean
import pandas as pd
import csv

dataH = pd.read_csv("multipleSclerosisOdeRatioERHealthy.csv", names=['ratioER'])

fig, ax = plt.subplots(figsize=(9,6))
ax.plot(range(0,2000),dataH['ratioER'],label="ratio Eff/Reg")
legend = ax.legend()
plt.title("2000 steps - 100 samples - linear scale - alpha_R=1.0")
#plt.savefig("MS_2000_h.png")
#plt.show()

fig, ax = plt.subplots(figsize=(9,6))
ax.set_yscale('log')
ax.plot(range(0,2000),dataH['ratioER'],label="ratio Eff/Reg")
legend = ax.legend()
plt.title("2000 steps - 100 samples - log scale - alpha_R=1.0")
#plt.savefig("MS_2000_h.png")
#plt.show()

dataS = pd.read_csv("multipleSclerosisOdeRatioERSick.csv", names=['ratioER'])

fig, ax = plt.subplots(figsize=(9,6))
ax.plot(range(0,2000),dataS['ratioER'],label="ratio Eff/Reg")
legend = ax.legend()
plt.title("2000 steps - 100 samples - linear scale - alpha_R=0.25")
#plt.savefig("MS_2000_h.png")
#plt.show()

fig, ax = plt.subplots(figsize=(9,6))
ax.set_yscale('log')
ax.plot(range(0,2000),dataS['ratioER'],label="ratio Eff/Reg")
legend = ax.legend()
plt.title("2000 steps - 100 samples - log scale - alpha_R=0.25")
#plt.savefig("MS_2000_h.png")
#plt.show()


data_c1 = pd.read_csv("runs_ms_sick.csv",usecols=[6],nrows=2000)
data_c2 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=2000,nrows=2000)
data_c3 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=4000,nrows=2000)
data_c4 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=6000,nrows=2000)
data_c5 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=8000,nrows=2000)
data_c6 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=10000,nrows=2000)
data_c7 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=12000,nrows=2000)
data_c8 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=14000,nrows=2000)
data_c9 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=16000,nrows=2000)
data_c10 = pd.read_csv("runs_ms_sick.csv",usecols=[6],skiprows=18000,nrows=2000)
data_ucl = pd.read_csv("runs_ms_sickuc.csv",usecols=['l'],nrows=2000)
data_ucL = pd.read_csv("runs_ms_sickuc.csv",usecols=['L'],nrows=2000)


fig, ax = plt.subplots(figsize=(9,6))
ax.set_yscale('log')
ax.plot(range(0,2000),data_c1,marker="o")
ax.plot(range(0,2000),data_c2,marker="+")
ax.plot(range(0,2000),data_c3,marker=".")
ax.plot(range(0,2000),data_c4,marker="o")
ax.plot(range(0,2000),data_c5,marker="+")
ax.plot(range(0,2000),data_c6,marker=".")
ax.plot(range(0,2000),data_c7,marker="o")
ax.plot(range(0,2000),data_c8,marker="+")
ax.plot(range(0,2000),data_c9,marker=".")
ax.plot(range(0,2000),data_c10,marker=".")
#legend = ax.legend()
plt.title("2000 steps - 100 samples - log scale - alpha_R=0.25")
#plt.savefig("prova.png")
plt.show()
