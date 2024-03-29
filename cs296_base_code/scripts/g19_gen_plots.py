import os
import numpy as np
from pylab import *
import matplotlib
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.mlab as mlab
import math
os.chdir("../")
os.system("mkdir -p plots")
os.system("rm -rf ./plots/*")
#Plot01
file='data/lab05_g19_data.csv'
r=mlab.csv2rec(file)

fig = Figure(figsize=(8,6))
canvas = FigureCanvas(fig)
ax = fig.add_subplot(111)
ax.set_title('StepTime, LoopTime(averaged over all reruns) v/s. Iterations')
ax.set_xlabel('Iteration Values')
ax.set_ylabel('StepTime/LoopTime averaged over all reruns')
#y1dict is getting the steptime avged over reruns,y2dict-->looptime,y3dict-->collision,y4dict-->velo,y5dict-->posn
y1dict={x:0 for x in range(1,101)}
y2dict={x:0 for x in range(1,101)}
y3dict={x:0 for x in range(1,101)}
y4dict={x:0 for x in range(1,101)}
y5dict={x:0 for x in range(1,101)}
#print(y5dict)
for x in range(1,101):
	li=filter(lambda a: a[1]==x, r)
	for j in li:
		y1dict[x] = y1dict[x] + j[2]
		y2dict[x] = y2dict[x] + j[6]
		y3dict[x] = y3dict[x] + j[3]
		y4dict[x] = y4dict[x] + j[4]
		y5dict[x] = y5dict[x] + j[5]
				
	y1dict[x] = y1dict[x]/100
	y2dict[x] = y2dict[x]/100
	y3dict[x] = y3dict[x]/100
	y4dict[x] = y4dict[x]/100
	y5dict[x] = y5dict[x]/100
	
#print(y5dict)	
y11dict=y1dict	

xlist=[]
y1list=[]
y2list=[]
y3list=[]
y4list=[]
y5list=[]

for x in range(1,101):
	xlist.append(x)
	y1list.append(y1dict[x])
	y2list.append(y2dict[x])
	y3list.append(y3dict[x])
	y4list.append(y4dict[x])
	y5list.append(y5dict[x])
	
#print(y5dict[80])		
ax.scatter(xlist,y1list,s=10,color='blue', marker="+",label="Step Time");
ax.scatter(xlist,y2list,s=10,color='green', marker="+",label="Loop Time");
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1],loc=7)
canvas.print_figure('./plots/g19_proj_plot01.png')

#plot02
fig = Figure(figsize=(11,10))
canvas = FigureCanvas(fig)
ax = fig.add_subplot(111)
ax.set_title('Various Times (averaged over all reruns) v/s. Iterations')
ax.set_xlabel('Iteration Values')
ax.set_ylabel('Step, Collision, Velocity Update and Position Update Times')
ax.scatter(xlist,y1list,s=10,color='blue', marker="+",label="Step Time");
ax.scatter(xlist,y3list,s=10,color='green', marker="+",label="Collision Time");
ax.scatter(xlist,y4list,s=10,color='tomato', marker="*",label="Velocity Updates Time");
ax.scatter(xlist,y5list,s=10,color='purple', marker="*",label="Position updates Time");
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1],loc=1)
canvas.print_figure('./plots/g19_proj_plot02.png')

#plot03,plot04
fig = Figure(figsize=(8,6))
canvas = FigureCanvas(fig)
ax = fig.add_subplot(111)
ax.set_title('StepTime, LoopTime(averaged over all iterations) v/s. Reruns')
ax.set_xlabel('Rerun Values')
ax.set_ylabel('StepTime/LoopTime averaged over all iterations')
#y1dict is getting the steptime avged over iter,y2dict-->looptime,y3dict-->collision,y4dict-->velo,y5dict-->posn
y1dict={x:0 for x in range(1,101)}
y2dict={x:0 for x in range(1,101)}
y3dict={x:0 for x in range(1,101)}
y4dict={x:0 for x in range(1,101)}
y5dict={x:0 for x in range(1,101)}

for x in range(1,101):
	li=filter(lambda a: a[0]==x, r)
	for j in li:
		y1dict[x] = y1dict[x] + j[2]
		y2dict[x] = y2dict[x] + j[6]
		y3dict[x] = y3dict[x] + j[3]
		y4dict[x] = y4dict[x] + j[4]
		y5dict[x] = y5dict[x] + j[5]
				
	y1dict[x] = y1dict[x]/100
	y2dict[x] = y2dict[x]/100
	y3dict[x] = y3dict[x]/100
	y4dict[x] = y4dict[x]/100
	y5dict[x] = y5dict[x]/100
		

xlist=[]
y1list=[]
y2list=[]
y3list=[]
y4list=[]
y5list=[]

for x in range(1,101):
	xlist.append(x)
	y1list.append(y1dict[x])
	y2list.append(y2dict[x])
	y3list.append(y3dict[x])
	y4list.append(y4dict[x])
	y5list.append(y5dict[x])
	
		
ax.scatter(xlist,y1list,s=10,color='blue', marker="+",label="Step Time");
ax.scatter(xlist,y2list,s=10,color='green', marker="+",label="Loop Time");
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1],loc=7)
canvas.print_figure('./plots/g19_proj_plot03.png')


fig = Figure(figsize=(11,10))
canvas = FigureCanvas(fig)
ax = fig.add_subplot(111)
ax.set_title('Various Times (averaged over all iterations) v/s. Reruns')
ax.set_xlabel('Reruns Values')
ax.set_ylabel('Step, Collision, Velocity Update and Position Update Times')
ax.scatter(xlist,y1list,s=10,color='blue', marker="+",label="Step Time");
ax.scatter(xlist,y3list,s=10,color='green', marker="+",label="Collision Time");
ax.scatter(xlist,y4list,s=10,color='tomato', marker="*",label="Velocity Updates Time");
ax.scatter(xlist,y5list,s=10,color='purple', marker="*",label="Position updates Time");
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1],loc=7)
canvas.print_figure('./plots/g19_proj_plot04.png')


#plot05
def sq(x):
	return (x*x)

y11dict={x:0 for x in range(1,101)}

for x in range(1,101):
	li=filter(lambda a: a[1]==x, r)
	for j in li:
		y11dict[x] = y11dict[x] + j[2]
					
	y11dict[x] = y11dict[x]/100



y1dict={x:0 for x in range(1,101)}
errorbar1=[]
y5list=[]

for x in range(1,101):
	li=filter(lambda a: a[1]==x, r)
	for j in li:
		y1dict[x] = y1dict[x] + sq(j[2] - y11dict[x])
		
	
	errorbar1.append(math.sqrt(y1dict[x]/100))
	y5list.append(y11dict[x])
	
figure(5)
errorbar(xlist, y5list, yerr=errorbar1, color='blue', label = 'Step Time')
legend(loc=7)
xlabel('Iteration Value')
ylabel('Step Time')
title('Step Time (along with Error Bars) v/s. Iterations')
plt.savefig('./plots/g19_proj_plot05.png')

#plot06
lif=filter(lambda a: a[1] == 28, r)
li=[]
for i in lif:
	li.append(i[2])
	
histo, bin_edges = np.histogram(li, bins=50, normed=False)
cumulative = np.cumsum(histo)

figure(6)
hist(li, bins=50, color='purple', rwidth=0.9, label='Step Time')
plot(bin_edges[1:], cumulative, color='green', linewidth=1.5,label='Cumulative Step Time')
legend(loc=7)
xlabel('Variation of Step Time')
ylabel('Number of Observations')
title('Frequency Plot of Variation of Step Time for Iteration Value = 28')
plt.savefig('./plots/g19_proj_plot06.png')



