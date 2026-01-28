#  STARK: Software Tool for the Analysis of Robustness in the unKnown environment
#
#                 Copyright (C) 2023.
#
#  See the NOTICE file distributed with this work for additional information
#  regarding copyright ownership.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#              http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
#  or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

#  STARK: Software Tool for the Analysis of Robustness in the unKnown environment
#
#
#  See the NOTICE file distributed with this work for additional information
#  regarding copyright ownership.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#              http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
#  or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import numpy.random as rnd
import matplotlib.pyplot as plt
import numpy
from pathlib import Path
from statistics import mean
import csv


BASE_DIR = Path(__file__).resolve().parents[4]
RESULTS_DIR = BASE_DIR / "results"
GRAPHS_DIR = BASE_DIR / "graphs"

# create directories if they do not exist
GRAPHS_DIR.mkdir(parents=True, exist_ok=True)

distance_Z1_ref = numpy.genfromtxt(RESULTS_DIR / "skorokhod_Z1_Old.csv", names=["ref_prot_Z1"])
distance_Z1_diff = numpy.genfromtxt(RESULTS_DIR / "skorokhod_Z1_Old_diff.csv", names=["diff_prot_Z1"])

distance_Z1 = numpy.genfromtxt(RESULTS_DIR / "skorokhod_Z1.csv", names=["d_prot_Z1"])
distance_Z2 = numpy.genfromtxt(RESULTS_DIR / "skorokhod_Z2.csv", names=["d_prot_Z2"])
distance_Z3 = numpy.genfromtxt(RESULTS_DIR / "skorokhod_Z3.csv", names=["d_prot_Z3"])

adistance_Z1 = numpy.genfromtxt(RESULTS_DIR / "atomic_Z1.csv", names=["ad_prot_Z1"])
adistance_Z2 = numpy.genfromtxt(RESULTS_DIR / "atomic_Z2.csv", names=["ad_prot_Z2"])
adistance_Z3 = numpy.genfromtxt(RESULTS_DIR / "atomic_Z3.csv", names=["ad_prot_Z3"])

# skorokhod distances z1 z2 z3
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],label="distance_Z1")
ax.plot(range(550,1000),distance_Z2['d_prot_Z2'][550:1000],label="distance_Z2")
ax.plot(range(550,1000),distance_Z3['d_prot_Z3'][550:1000],label="distance_Z3")
legend = ax.legend(loc='upper right')
plt.savefig(GRAPHS_DIR / "distances.png")
plt.show()


# skorokhod vs Old Z1
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],label="revised")
ax.plot(range(550,1000),distance_Z1_ref['ref_prot_Z1'][550:1000],label="old")
legend = ax.legend(loc='upper right')
plt.savefig(GRAPHS_DIR / "SkorokhodOldDistancesZ1.png")

# Old diff
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1_diff['diff_prot_Z1'][550:1000],label="diff")
legend = ax.legend(loc='upper right')
plt.savefig(GRAPHS_DIR / "SkorokhodOldDiff.png")


# atomic vs skorokhod Z1
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z1['ad_prot_Z1'][550:1000],label="atomic")
legend = ax.legend(loc='upper right')
plt.savefig(GRAPHS_DIR / "SkorokhodAtomicDistancesZ1.png")

# atomic vs skorokhod Z2
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z2['d_prot_Z2'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z2['ad_prot_Z2'][550:1000],label="atomic")
legend = ax.legend(loc='upper right')
plt.savefig(GRAPHS_DIR / "SkorokhodAtomicDistancesZ2.png")

# atomic vs skorokhod Z3
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z3['d_prot_Z3'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z3['ad_prot_Z3'][550:1000],label="atomic")
legend = ax.legend(loc='upper right')
plt.savefig(GRAPHS_DIR / "SkorokhodAtomicDistancesZ3.png")

# skorokhod vs skorokhod Z1
fix, ax = plt.subplots()
ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],label="Skorokhod")
ax.plot(range(550,1000),adistance_Z1['ad_prot_Z1'][550:1000],label="atomic")
# ax.plot(range(550,1000),dist_sk['distance'][550:1000],label="SkorokhodSK")
legend = ax.legend(loc='upper right')
plt.savefig(GRAPHS_DIR / "SkorokhodSkorokhodDistancesZ1.png")



offset_Z1 = numpy.genfromtxt(RESULTS_DIR / "offsets_Z1.csv", names=["offsets_prot_Z1"])
offset_Z2 = numpy.genfromtxt(RESULTS_DIR / "offsets_Z2.csv", names=["offsets_prot_Z2"])
offset_Z3 = numpy.genfromtxt(RESULTS_DIR / "offsets_Z3.csv", names=["offsets_prot_Z3"])

# DistancesZ1 w offsets
fix, ax = plt.subplots()
line1, = ax.plot(range(550,1000),distance_Z1['d_prot_Z1'][550:1000],'b:',label="distance_Z1")
ax.set_ylabel('distance_Z1', color='b')
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(550,1000),offset_Z1["offsets_prot_Z1"][550:1000],'r--',label="offset_Z1")
ax2.set_ylabel('offsets_Z1', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig(GRAPHS_DIR / "distanceZ1.png")

# DistancesZ2 w offsets
fix, ax = plt.subplots()
line1, = ax.plot(range(550,1000),distance_Z2['d_prot_Z2'][550:1000],'b:',label="distance_Z2")
ax.set_ylabel('distance_Z2', color='b')
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(550,1000),offset_Z2["offsets_prot_Z2"][550:1000],'r--',label="offset_Z2")
ax2.set_ylabel('offsets_Z2', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig(GRAPHS_DIR / "distanceZ2.png")

# DistancesZ3 w offsets
fix, ax = plt.subplots()
line1, = ax.plot(range(550,1000),distance_Z3['d_prot_Z3'][550:1000],'b:',label="distance_Z3")
ax.set_ylabel('distance_Z2', color='b')
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(550,1000),offset_Z3["offsets_prot_Z3"][550:1000],'r--',label="offset_Z3")
ax2.set_ylabel('offsets_Z3', color='r')
lines = [line1, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper left')
plt.savefig(GRAPHS_DIR / "distanceZ3.png")

Protein_Z1 = numpy.genfromtxt(RESULTS_DIR / "new_plotZ1.csv", names=["prot_Z1"])
Protein_Z2 = numpy.genfromtxt(RESULTS_DIR / "new_plotZ2.csv", names=["prot_Z2"])
Protein_Z3 = numpy.genfromtxt(RESULTS_DIR / "new_plotZ3.csv", names=["prot_Z3"])

pProtein_Z1 = numpy.genfromtxt(RESULTS_DIR / "new_pplotZ1.csv", names=["pprot_Z1"])
pProtein_Z2 = numpy.genfromtxt(RESULTS_DIR / "new_pplotZ2.csv", names=["pprot_Z2"])
pProtein_Z3 = numpy.genfromtxt(RESULTS_DIR / "new_pplotZ3.csv", names=["pprot_Z3"])

# Z1 states
fix, ax = plt.subplots()
line2, = ax.plot(range(0,1000),pProtein_Z1['pprot_Z1'],'g', label="perturbed_Z1")
line3, = ax.plot(range(0,1000),Protein_Z1['prot_Z1'],'orange',label="nominal_Z1")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='lower right')
plt.savefig(GRAPHS_DIR / "statesZ1.png")

# Z2 states
fix, ax = plt.subplots()
line2, = ax.plot(range(0,1000),pProtein_Z2['pprot_Z2'],'g', label="perturbed_Z2")
line3, = ax.plot(range(0,1000),Protein_Z2['prot_Z2'],'orange',label="nominal_Z2")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='lower right')
plt.savefig(GRAPHS_DIR / "statesZ2.png")

# Z3 states
fix, ax = plt.subplots()
line2, = ax.plot(range(0,1000),pProtein_Z3['pprot_Z3'],'g', label="perturbed_Z3")
line3, = ax.plot(range(0,1000),Protein_Z3['prot_Z3'],'orange',label="nominal_Z3")
lines = [line2, line3]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='lower right')
plt.savefig(GRAPHS_DIR / "statesZ3.png")


distance_Z1 = numpy.genfromtxt(RESULTS_DIR / "skorokhod_Z1.csv", names=["d_prot_Z1"])
fix, ax = plt.subplots()
line1, = ax.plot(range(0,1000),distance_Z1['d_prot_Z1'],'b:',label="distance_Z1")
ax.set_ylabel('distance_Z1', color='b')
# create y-axis that shares x-axis
ax1 = ax.twinx()
line2, = ax1.plot(range(0,1000),pProtein_Z1['pprot_Z1'],'g', label="perturbed_Z1")
line3, = ax1.plot(range(0,1000),Protein_Z1['prot_Z1'],'orange',label="nominal_Z1")
ax1.set_ylabel('nom/perturbed_Z1', color='g')
offset_Z1 = numpy.genfromtxt(RESULTS_DIR / "offsets_Z1.csv", names=["offsets_prot_Z1"])
# create y-axis that shares x-axis
ax2 = ax.twinx()
line4, = ax2.plot(range(0,1000),offset_Z1["offsets_prot_Z1"],'r--',label="offset_Z1")
ax2.set_ylabel('offsets_Z1', color='r')
lines = [line1, line2, line3, line4]
labels = [line.get_label() for line in lines]
ax.legend(lines, labels, loc='upper right')

plt.savefig(GRAPHS_DIR / "fullZ1.png")


fix, ax = plt.subplots()
ThreshOld = []
Value = []

with open(RESULTS_DIR / "evalR.csv",'r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        ThreshOld.append(row[0])
        Value.append(row[1])

plt.scatter(ThreshOld, Value, color = 'b',s = 100)
plt.xticks(rotation = 0)
plt.xlabel('ThreshOld')
plt.ylabel('Evaluation (0.0 = True, 1.0 = False)')
plt.title('Robustness', fontsize = 20)
plt.savefig(GRAPHS_DIR / "robustness.png")

plt.show