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
		self.fig = plt.figure(figsize=(8,6))
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
		ax1.plot([d[0] for d in data],[d[7] for d in data],color="gray",label="APE")
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
		
	def calc_max(self,data):
		return max([d[7] for d in data])
	
	def calc_min(self,data):
		return min([d[7] for d in data])

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
		self.time_limit = 1200
		self.load_data()
		self.graph = Graph()
		self.graph.add_data(self.file)

	def __del__(self):
		self.csv_file.close()

	def load_data(self):
		if 2 <= len(self.args):
			print("load file: " + self.args[1])
			self.csv_file = open(self.args[1],"r",encoding="ms932")
			file = csv.reader(self.csv_file,delimiter=",",doublequote=True,lineterminator="\r\n",quotechar='"',skipinitialspace=True)
		else: 
			print('Arguments are too short')
			return 

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
