#!/usr/bin/env python3

from cProfile import label
from cmath import sqrt
from tracemalloc import start
from turtle import color

from matplotlib.patches import bbox_artist
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import csv
import sys

class Graph:
	def __init__(self):
		self.fig = plt.figure(figsize=(20,8))
		plt.rcParams["xtick.direction"] = "in"
		plt.rcParams["ytick.direction"] = "in"
	
	def add_data(self,data):
		print(" max: ", self.calc_max(data))
		print(" min: ", self.calc_min(data))
		print("mean: ", self.calc_mean(data))
		print("rmse: ", self.calc_rmse(data))
		print(" std: ", self.calc_std(data))

		ax1 = self.fig.add_subplot(2,1,1)
		# ax1.set_title('Absolute Pose Error')

		# xlabel
		ax1.set_xlabel('t [s]')
		ax1.set_xlim(0.0,1200.0)
		ax1.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
	
		# ylabel
		ax1.set_ylabel('APE [m]')
		ax1.set_ylim(0.0,5.1)
		ax1.set_yticks(np.arange(0.00,5.1,step=1.00))
		ax1.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.3f'))
		ax1.plot((d[0] for d in data),(d[7] for d in data),color="gray",label="APE")
		mean = self.calc_mean(data)
		std = self.calc_std(data)
		# ax1.fill_between([d[0] for d in data],mean + std,mean - std,alpha=0.2,color="gray")
		ax1.plot([d[0] for d in data],[mean for d in data],color="red",label="Mean")
		ax1.plot([d[0] for d in data],[self.calc_median(data) for d in data],color="blue",label="Median")
		ax1.plot([d[0] for d in data],[self.calc_rmse(data) for d in data],color="green",label="rmse")
		ax1.legend(loc='upper left',bbox_to_anchor=(1,1))
		self.fig.tight_layout()

		ax2 = self.fig.add_subplot(2,1,2)
		ax2.set_xlabel('X [m]')
		ax2.set_ylabel('Y [m]')
		# ax2.set_title('Trajectory')
		ax2.plot([d[2] for d in data],[d[1] for d in data],color="blue",label="Estimated_Pose")
		ax2.plot([d[5] for d in data],[d[4] for d in data],color="orange",label="Ground_Truth")
		ax2.legend(loc='upper left',bbox_to_anchor=(1,1))
		self.fig.tight_layout()

		self.show()
	
	def add_data2(self,data,obs_data):
		print(" max: ", self.calc_max(data))
		print(" min: ", self.calc_min(data))
		print("mean: ", self.calc_mean(data))
		print("rmse: ", self.calc_rmse(data))
		print(" std: ", self.calc_std(data))

		ax1 = self.fig.add_subplot(2,2,1)
		# ax1.set_title('Absolute Pose Error')

		# xlabel
		ax1.set_xlabel('t [s]')
		ax1.set_xlim(0.0,1200.0)
		ax1.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
	
		# ylabel
		ax1.set_ylabel('APE [m]')
		ax1.set_ylim(0.0,5.1)
		ax1.set_yticks(np.arange(0.00,5.1,step=1.00))
		ax1.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.3f'))
		ax1.plot([d[0] for d in data],[d[7] for d in data],color="gray",label="APE")
		mean = self.calc_mean(data)
		std = self.calc_std(data)
		# ax1.fill_between([d[0] for d in data],mean + std,mean - std,alpha=0.2,color="gray")
		ax1.plot([d[0] for d in data],[mean for d in data],color="red",label="Mean")
		ax1.plot([d[0] for d in data],[self.calc_median(data) for d in data],color="blue",label="Median")
		ax1.plot([d[0] for d in data],[self.calc_rmse(data) for d in data],color="green",label="rmse")
		ax1.legend(loc='upper left',bbox_to_anchor=(1,1))
		self.fig.tight_layout()

		ax2 = self.fig.add_subplot(2,2,2)
		ax2.set_xlabel('X [m]')
		ax2.set_ylabel('Y [m]')
		# ax2.set_title('Trajectory')
		# ax2.plot([d[2] for d in data],[-1*d[1] for d in data],color="blue",label="Estimated_Pose")
		# ax2.plot([d[5] for d in data],[-1*d[4] for d in data],color="orange",label="Ground_Truth")
		ax2.scatter([d[2] for d in data],[-1*d[1] for d in data],color="blue",label="Estimated_Pose",s=10)
		ax2.scatter([d[5] for d in data],[-1*d[4] for d in data],color="orange",label="Ground_Truth",s=10)
		
		ax2.legend(loc='upper left',bbox_to_anchor=(1,1))
		self.fig.tight_layout()

		ax3 = self.fig.add_subplot(2,2,3)
		# x-label
		ax3.set_xlabel('t [s]')
		ax3.set_xlim(0.0,1200.0)
		ax3.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))
		
		# y-label
		y_ticks_labels = ['','bench','big_bench','chair','elevator','fire_extinguisher','fire_hydrant','kitchenette_icon','table','toilet_icon','trash_can','roomba']
		tmp_y_ticks_labels = ['','bench','big bench','chair','elevator','fire\nextinguisher','fire hydrant','kitchenette\nicon','table','toilet icon','trash can','roomba']
		ax3.set_yticks(np.arange(0,1,1.0/(len(y_ticks_labels))))
		ax3.set_yticklabels(tmp_y_ticks_labels,rotation='horizontal',fontsize=8)
		
		for y_list in y_ticks_labels:
			x = []
			y = []
			for obs_d in obs_data:
				if y_list == obs_d[1]:
					x.append(obs_d[0])
					y.append(y_ticks_labels.index(obs_d[1])*1.0/(len(y_ticks_labels)))
			ax3.scatter(x,y,s=5)
		
		time_list = [d[0] for d in obs_data]
		new_time_list = sorted(time_list)
		# no_obs_list = []
		for i in range(len(new_time_list)):
			if i != len(new_time_list) - 1:
				diff = new_time_list[i+1] - new_time_list[i]
				if(diff > 15.0): 
					ax3.axvline(x=new_time_list[i],linestyle="dashed")
					ax3.axvline(x=new_time_list[i+1],linestyle="dashed")

		# y_ticks_labels = ['','Static','Semi-Dynamic','Robot']
		# ax3.set_yticks(np.arange(0.0,1.0,1.0/(len(y_ticks_labels))))
		# ax3.set_yticklabels(y_ticks_labels,rotation='horizontal',fontsize=8)
		# x1 = []
		# x2 = []
		# x3 = []
		# y1 = []
		# y2 = []
		# y3 = []
		# for y_list in y_ticks_labels:
		# 	for obs_d in obs_data:
		# 		if obs_d[1] == 'big_bench' or obs_d[1] == 'elevator' or obs_d[1] == 'fire_extinguisher' or obs_d[1] == 'kitchenette_icon' or obs_d[1] == 'toilet_icon':
		# 			x1.append(obs_d[0])
		# 			y1.append(y_ticks_labels.index('Static')*1.0/(len(y_ticks_labels)))
		# 		elif obs_d[1] == 'bench' or obs_d[1] == 'chair' or obs_d[1] == 'fire_hydrant' or obs_d[1] == 'table' or obs_d[1] == 'trash_can':
		# 			x2.append(obs_d[0])
		# 			y2.append(y_ticks_labels.index('Semi-Dynamic')*1.0/(len(y_ticks_labels)))
		# 		elif obs_d[1] == 'roomba':
		# 			x3.append(obs_d[0])
		# 			y3.append(y_ticks_labels.index('Robot')*1.0/(len(y_ticks_labels)))

		# ax3.scatter(x1,y1)
		# ax3.scatter(x2,y2)
		# ax3.scatter(x3,y3)
		ax3.legend()
		self.fig.tight_layout()

		self.show()
		self.fig.savefig("img.png")

	def calc_max(self,data):
		return max((d[7] for d in data))
	
	def calc_min(self,data):
		return min((d[7] for d in data))

	def calc_mean(self,data):
		return np.mean([d[7] for d in data])

	def calc_median(self,data):
		return np.median([d[7] for d in data])

	def calc_rmse(self,data):
		# mean = self.calc_mean(data)
		error = 0.0
		for d in data:
			error += d[7]*d[7]	
		return np.sqrt(error/len(data))

	def calc_std(self,data):
		return np.std([d[7] for d in data])

	def show(self):
		plt.show()
	
	def close(self):
		plt.clf()
		plt.close()

class DataViwer:
	def __init__(self,args):
		self.args = args
		self.time_limit = 2400
		self.load_data()
		self.graph = Graph()
		# self.graph.add_data(self.file)
		self.graph.add_data2(self.file,self.obs_file)

	def __del__(self):
		self.csv_file.close()

	def load_data(self):
		if 2 <= len(self.args):
			print("load file: " + self.args[1])
			self.csv_file = open(self.args[1],"r",encoding="ms932")
			file = csv.reader(self.csv_file,delimiter=",",doublequote=True,lineterminator="\r\n",quotechar='"',skipinitialspace=True)

			self.file = []
			is_first = True
			for row in file:
				if is_first: 
					start_time = float(row[0])
					is_first = False
				time = float(row[0]) - start_time
				if(time <= self.time_limit):
					error_x = float(row[1]) - float(row[4])
					error_y = float(row[2]) - float(row[5])
					ape = (np.sqrt(error_x*error_x + error_y*error_y))
					self.file.append([time, 
				                      float(row[1]), float(row[2]), float(row[3]),
								      float(row[4]), float(row[5]), float(row[6]),
								      ape])
		else: 
			print('Arguments are too short')
			return 

		if 3 == len(self.args):
			print("load file: " + self.args[2])
			self.obs_csv_file = open(self.args[2],"r",encoding="ms932")
			file = csv.reader(self.obs_csv_file,delimiter=",",doublequote=True,lineterminator="\r\n",quotechar='"',skipinitialspace=True)
			
			self.obs_file = []
			for row in file:
				self.obs_file.append([float(row[1]),row[0]])


		# for debug
		# for row in self.file: print(row)

	def calc_process():
		pass

	def calc_ape(self):
		pass

	def process(self):
		pass

if __name__=='__main__':
	data_viwer = DataViwer(sys.argv)
	# data_viwer.load_data()
