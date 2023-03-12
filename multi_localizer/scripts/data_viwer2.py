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
        # plt.rcParams["figure.subplot.left"] = 0.01
        # plt.rcParams["figure.subplot.right"] = 0.95
        # plt.rcParams["figure.subplot.bottom"] = 0.2
        # plt.rcParams["figure.subplot.top"] =  0.9
        # plt.rcParams["figure.subplot.wspace"] = 0.34
        # plt.rcParams["figure.subplot.hspace"] = 0.20

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
        ax1.set_xlim(0.0,2400.0)
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
        # ax1.plot([d[0] for d in data],[mean for d in data],color="red",label="Mean")
        # ax1.plot([d[0] for d in data],[self.calc_median(data) for d in data],color="blue",label="Median")
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
        print(" pos_max: ", self.calc_pos_max(data))
        print(" pos_min: ", self.calc_pos_min(data))
        print("pos_mean: ", self.calc_pos_mean(data))
        print("pos_rmse: ", self.calc_pos_rmse(data))
        print(" pos_std: ", self.calc_pos_std(data))

        ax1 = self.fig.add_subplot(2,2,1)
        self.fig.text(0.18,0.51,'(a) Positional error',fontsize=14)
        ax1.set_xlabel('t [s]')
        ax1.set_xlim(0.0,2400.1)
        ax1.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))

        # ylabel
        ax1.set_ylabel('APE [m]')
        ax1.set_ylim(0.0,10.1)
        ax1.set_yticks(np.arange(0.00,10.1,step=1.00))
        ax1.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.3f'))
        ax1.plot([d[0] for d in data],[d[7] for d in data],color="gray",label="APE")
        ax1.plot([d[0] for d in data],[self.calc_pos_rmse(data) for d in data],color="green",label="RMSE")
        ax1.legend(loc='upper left',bbox_to_anchor=(1,1))
        self.fig.tight_layout()

        # print(" ori_max: ", self.calc_ori_max(data))
        # print(" ori_min: ", self.calc_ori_min(data))
        # print("ori_mean: ", self.calc_ori_mean(data))
        # print("ori_rmse: ", self.calc_ori_rmse(data))
        # print(" ori_std: ", self.calc_ori_std(data))
        print(" ori_max: ", self.calc_ori_max(data)*180.0/np.pi)
        print(" ori_min: ", self.calc_ori_min(data)*180.0/np.pi)
        print("ori_mean: ", self.calc_ori_mean(data)*180.0/np.pi)
        print("ori_rmse: ", self.calc_ori_rmse(data)*180.0/np.pi)
        print(" ori_std: ", self.calc_ori_std(data)*180.0/np.pi)

        ax2 = self.fig.add_subplot(2,2,2)
        self.fig.text(0.65,0.51,'(b) Orientational error',fontsize=14)
        ax2.set_xlabel('t [s]')
        ax2.set_xlim(0.0,2400.1)
        ax2.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))

        # ylabel
        ax2.set_ylabel('APE [rad]')
        ax2.set_ylim(0.0,4.1)
        ax2.set_yticks(np.arange(0.00,4.1,step=1.00))
        ax2.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.3f'))
        ax2.plot([d[0] for d in data],[d[8] for d in data],color="gray",label="APE")
        ax2.plot([d[0] for d in data],[self.calc_ori_rmse(data) for d in data],color="green",label="RMSE")
        ax2.legend(loc='upper left',bbox_to_anchor=(1,1))
        self.fig.tight_layout()

        # ax2.set_ylabel('APE [deg]')
        # ax2.set_ylim(0.0,180.1)
        # ax2.set_yticks(np.arange(0.0,180.1,step=30.0))
        # ax2.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.2f'))
        # ax2.plot([d[0] for d in data],[d[8]*180.0/np.pi for d in data],color="gray",label="APE")
        # ax2.plot([d[0] for d in data],[self.calc_ori_deg_rmse(data) for d in data],color="green",label="RMSE")
        # ax2.legend(loc='upper left',bbox_to_anchor=(1,1))
        # self.fig.tight_layout()

        ax3 = self.fig.add_subplot(2,2,3)
        self.fig.text(0.18,0.02,'(c) Observed objects',fontsize=14)
        ax3.set_xlabel('t [s]')
        ax3.set_xlim(0.0,2500.0)
        ax3.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.0f'))

        # y-label
        # y_ticks_labels = ['','bench','big_bench','chair','elevator','fire_extinguisher','fire_hydrant','kitchenette_icon','toilet_icon','trash_can','roomba']
        # y_ticks_labels_color = ["","red","yellow","lightcoral","purple","aqua","blue","fuchsia","darkgreen","lawngreen","black"]
        # tmp_y_ticks_labels = ['','bench','big bench','chair','elevator','fire\nextinguisher','fire hydrant','kitchenette\nicon','toilet icon','trash can','roomba']
        y_ticks_labels = ['','bench','big_bench','chair','elevator','fire_extinguisher','fire_hydrant','kitchenette_icon','toilet_icon','trash_can']
        y_ticks_labels_color = ["","red","yellow","lightcoral","purple","aqua","blue","fuchsia","darkgreen","lawngreen"]
        tmp_y_ticks_labels = ['','bench','big bench','chair','elevator','fire\nextinguisher','fire hydrant','kitchenette\nicon','toilet icon','trash can']
        ax3.set_yticks(np.arange(0,1,1.0/(len(y_ticks_labels))))
        ax3.set_yticklabels(tmp_y_ticks_labels,rotation='horizontal',fontsize=8)

        for y_list in y_ticks_labels:
            x = []
            y = []
            for obs_d in obs_data:
                if y_list == obs_d[1]:
                    # if obs_d[0] < 600.0:
                        x.append(obs_d[0])
                        y.append(y_ticks_labels.index(obs_d[1])*1.0/(len(y_ticks_labels)))
                        ax3.scatter(x,y,s=5,c=y_ticks_labels_color[y_ticks_labels.index(obs_d[1])])
                    # x.append(obs_d[0])
                    # y.append(y_ticks_labels.index(obs_d[1])*1.0/(len(y_ticks_labels)))
                    # ax3.scatter(x,y,s=5,c=y_ticks_labels_color[y_ticks_labels.index(obs_d[1])])

        # time_list = [d[0] for d in obs_data]
        # new_time_list = sorted(time_list)
        # # for n in new_time_list: print(n)
        # # no_obs_list = []
        # for i in range(len(new_time_list)):
        #     if i != len(new_time_list) - 1:
        #         diff = new_time_list[i+1] - new_time_list[i]
        #         if(diff > 10.0):
        #             print(new_time_list[i])
        #             print(new_time_list[i+1])
        #             ax3.axvline(x=new_time_list[i],linestyle="dashed")
        #             ax3.axvline(x=new_time_list[i+1],linestyle="dashed")
        ax3.legend()
        self.fig.tight_layout()

        ax4 = self.fig.add_subplot(2,2,4)
        self.fig.text(0.65,0.02,'(d) Estimated trajectories',fontsize=14)
        ax4.set_xlabel('X [m]')
        ax4.set_ylabel('Y [m]')
        ax4.scatter([d[2] for d in data],[-1*d[1] for d in data],color="blue",  label="Estimated_Pose",s=5)
        ax4.scatter([d[5] for d in data],[-1*d[4] for d in data],color="orange",label="Ground_Truth",  s=5)
        
        ax4.legend(loc='upper left',bbox_to_anchor=(1,1))
        self.fig.tight_layout()

        self.fig.subplots_adjust(left=0.05,bottom=0.10,right=0.90,top=0.97, wspace=0.25,hspace=0.25)
        # self.show()
        plt.show()
        self.fig.savefig("img.png")
        # self.close()

    def calc_pos_max(self,data):
        return max((d[7] for d in data))

    def calc_ori_max(self,data):
        return max((d[8] for d in data))

    def calc_pos_min(self,data):
        return min((d[7] for d in data))

    def calc_ori_min(self,data):
        return min((d[8] for d in data))

    def calc_pos_mean(self,data):
        return np.mean([d[7] for d in data])

    def calc_ori_mean(self,data):
        return np.mean([d[8] for d in data])

    def calc_pos_median(self,data):
        return np.median([d[7] for d in data])

    def calc_ori_median(self,data):
        return np.median([d[8] for d in data])

    def calc_pos_rmse(self,data):
        # mean = self.calc_mean(data)
        error = 0.0
        for d in data:
            error += d[7]*d[7]
        return np.sqrt(error/len(data))

    def calc_ori_rmse(self,data):
        # mean = self.calc_mean(data)
        error = 0.0
        for d in data:
            error += d[8]*d[8]
            # error += d[8]*180.0/np.pi*d[8]*180.0/np.pi
        return np.sqrt(error/len(data))

    def calc_ori_deg_rmse(self,data):
        # mean = self.calc_mean(data)
        error = 0.0
        for d in data:
            # error += d[8]*d[8]
            deg = d[8]*180.0/np.pi
            error += deg*deg
        return np.sqrt(error/len(data))

    def calc_pos_std(self,data):
        return np.std([d[7] for d in data])

    def calc_ori_std(self,data):
        return np.std([d[8] for d in data])

    # def show(self):
        # plt.show()

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
                    error_theta = self.calc_angle_diff(float(row[3]),float(row[6]))
                    pos_ape = (np.sqrt(error_x*error_x + error_y*error_y))
                    ori_ape = (np.sqrt(error_theta*error_theta))
                    self.file.append([time,
                                      float(row[1]), float(row[2]), float(row[3]),
                                      float(row[4]), float(row[5]), float(row[6]),
                                      pos_ape,ori_ape])
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

    def calc_angle_diff(self,a,b):
        a_angle = np.arctan2(np.sin(a),np.cos(a))
        b_angle = np.arctan2(np.sin(b),np.cos(b))

        d1 = a_angle - b_angle
        d2 = 2.0*np.pi - np.fabs(d1)

        if d1 > 0: d2 *= -1.0
        if np.fabs(d1) < np.fabs(d2): return d1
        else: return d2

    def calc_process():
        pass

    def calc_ape(self):
        pass

    def process(self):
        pass

if __name__=='__main__':
    data_viwer = DataViwer(sys.argv)
    # data_viwer.load_data()
