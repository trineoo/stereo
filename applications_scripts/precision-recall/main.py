#!/usr/bin/ python
# coding=utf-8
import os
import numpy as np
import matplotlib.pyplot as plt
import subprocess
import matplotlib.cm as cm
minO = 0.1

for minOverlap in np.arange(minO, 1, 0.05):
    prec= [1]
    recall = [0]
    thres = [0]
    for threshold in np.arange(0, 1, 0.01):
	command = 'python precision-recall.py ' + str(minOverlap) + ' ' + str(threshold) + ' -na'+ ' -np'
	result = subprocess.check_output(command, shell=True)
	for line in result.split('\n'):
	    words = line.split(' ')
  	    if words[0] == 'Precision:':
		prec.append(float(words[1]))
	    if words[0] == 'Recall:':
		recall.append(float(words[1]))
		thres.append(threshold)

    prec = np.array(prec)
    recall = np.array(recall)
    thres = np.array(thres)
    iou = np.full((1,len(thres)),str("{:.2f}".format(minOverlap)))
    stack = np.vstack((recall, prec, thres, iou)).T
    stack = stack[np.argsort(stack[:, 0])]
 
    if minOverlap == float(minO):
	total = np.matrix(stack)
    else:
	total = np.concatenate((total,stack),axis=1)	
    print("Calculated IoU >= " + str("{:.2f}".format(minOverlap)))


title = "Precision Recall Curve for Boat"
plt.title(title) 
plt.xlabel("Recall") 
plt.ylabel("Precision")
plt.axis([0, 1, 0, 1.05])
colors = cm.rainbow(np.linspace(0, 1, np.size(total,1)/4))
for i in range(0, np.size(total,1),4):
    plt.plot(total[:,i],total[:,i+1],'-', color=colors[i/4], label=total[0,i+3])

plt.legend(title='IoU>=', bbox_to_anchor=(1.0, 1.0))
plt.tight_layout()
plt.show()


