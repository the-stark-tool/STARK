# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""


import matplotlib.pyplot as plt
import numpy



a = numpy.genfromtxt("rs_running_sample_a.csv", names=["a"])
b = numpy.genfromtxt("rs_running_sample_b.csv", names=["b"])
c = numpy.genfromtxt("rs_running_sample_c.csv", names=["c"])
d = numpy.genfromtxt("rs_running_sample_d.csv", names=["d"])

fix, ax = plt.subplots()
ax.plot(range(0,10),a['a'],color='g',linestyle='dotted',label="a")
ax.plot(range(0,10),b['b'],color='lightskyblue',label="b")
ax.plot(range(0,10),c['c'],color='b',label="c")
ax.plot(range(0,10),d['d'],color='orange',linestyle='dashed',label="d")
legend = ax.legend()
plt.savefig("running_ex_res_plot.png")
plt.show()



aa = numpy.genfromtxt("rs_running_ag_a.csv", names=["aa"])
ab = numpy.genfromtxt("rs_running_ag_b.csv", names=["ab"])
ac = numpy.genfromtxt("rs_running_ag_c.csv", names=["ac"])
ad = numpy.genfromtxt("rs_running_ag_d.csv", names=["ad"])

fix, ax = plt.subplots()
ax.plot(range(0,8),aa['aa'],color='g',linestyle='dotted',label="a")
ax.plot(range(0,8),ab['ab'],color='lightskyblue',label="b")
ax.plot(range(0,8),ac['ac'],color='b',label="c")
ax.plot(range(0,8),ad['ad'],color='orange',linestyle='dashed',label="d")
legend = ax.legend()
plt.savefig("running_ex_res_ag.png")
plt.show()



pa = numpy.genfromtxt("rs_running_p_a.csv", names=["pa"])
pb = numpy.genfromtxt("rs_running_p_b.csv", names=["pb"])
pc = numpy.genfromtxt("rs_running_p_c.csv", names=["pc"])
pd = numpy.genfromtxt("rs_running_p_d.csv", names=["pd"])

fix, ax = plt.subplots()
ax.plot(range(0,8),pa['pa'],color='g',linestyle='dotted',label="a")
ax.plot(range(0,8),pb['pb'],color='lightskyblue',label="b")
ax.plot(range(0,8),pc['pc'],color='b',label="c")
ax.plot(range(0,8),pd['pd'],color='orange',linestyle='dashed',label="d")
legend = ax.legend()
plt.savefig("running_ex_res_pert.png")
plt.show()