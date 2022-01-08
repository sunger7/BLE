from django.apps import AppConfig
import numpy as np
from scipy.signal import find_peaks
import json
import logging
logger =  logging.getLogger('Localizate')
class ApiConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'API'

class ThreePointLoc():
    def __init__(self, BLE_MAP_FILE):
        self.beacon = []
        with open(BLE_MAP_FILE) as f:
            data = f.readlines()
            index = 0
            for i in range(len(data)):
                data[i] = data[i].rstrip("\n")
                splits = data[i].split(',')
                #print(splits)
                if ( len(splits)<5 ):
                    continue
                else:
                    self.beacon.append(splits)
                    
        #beacon x y 
        self.Beacon_xy = np.zeros((len(self.beacon),2))
        for i in range(self.Beacon_xy.shape[0]):
            self.Beacon_xy[i,0] = self.beacon[i][3]
            self.Beacon_xy[i,1] = self.beacon[i][4]
        self.beacon_dic = self.Convert(self.beacon)    
        #evalkk=np.mat([[1637],[1219]])#当前卡尔曼估计值
        #evalkk1=np.mat([[1628],[1211]])#当前预测值
        #self.evalk1k1=np.mat([[0],[0]])#上一次卡尔曼估计值
        #self.Pkk=np.mat([[100,0],[0,100]])
        #self.Pkk1=np.mat([[100,0],[0,100]])
        #self.Pk1k1=np.mat([[100,0],[0,100]])
        #self.Q = np.mat([[100,0],[0,100]])
        #self.R = np.mat([[200,0],[0,200]])
        
        
    def Load_beacon_seq(self,realtime_json):
        res_sparse = json.loads(realtime_json)
        if type(res_sparse) == 'str':
            res_sparse = json.loads(res_sparse) 
        #logger.info((res_sparse[-1]))
            
        #time_seq_beacon = []
        #time_seq_compass = []
        #time_seq_accelerometer = []
        #time_seq_deviceMotion = []
        #beacon_seq = []
        #compass_seq = []
        #accelerometer_seq = []
        #deviceMotion_seq = []
        last_loc = []
        last_kalman = []
        time_samples_beacon = []
        time_samples_compass = []
        time_samples_accelerometer = []
        time_samples_deviceMotion = []
        beacon_response = []
        compass_samples = []
        accelerometer_samples = []
        deviceMotion_samples = []        
        for res in res_sparse:           
            if res['type'] == 'ibeacon':
                for one_beacon in res['res']:                    
                    beacon_response.append(one_beacon)
                    time_samples_beacon.append(res['timestamp'])
            if res['type'] == 'compass':
                #解析罗盘
                compass_samples.append(res['res'])
                time_samples_compass.append(res['timestamp'])
            if res['type'] == 'accelerometer':
                #解析加速器
                data = res['res']
                accelerometer_samples.append(np.sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]))
                time_samples_accelerometer.append(res['timestamp'])
            if res['type'] == 'deviceMotion':
                #解析设备方向
                deviceMotion_samples.append(res['res'])
                time_samples_deviceMotion.append(res['timestamp'])
            if res['type'] == 'last_loc':
                last_loc = res['res']
            if res['type'] == 'last_kalman':
                last_kalman = res['res']
        
        #beacon_seq.append(beacon_response)
        #time_seq_beacon.append(time_samples_beacon)
        #compass_seq.append(compass_samples)
        #time_seq_compass.append(time_samples_compass)
        #accelerometer_seq.append(accelerometer_samples)
        #time_seq_accelerometer.append(time_samples_accelerometer)
        #deviceMotion_seq.append(deviceMotion_samples)
        #time_seq_deviceMotion.append(time_samples_deviceMotion)
        #logger.info(beacon_seq)
        return beacon_response, time_samples_beacon, compass_samples, time_samples_compass,accelerometer_samples,time_samples_accelerometer, deviceMotion_samples,time_samples_deviceMotion,last_loc,last_kalman
        
    def Distance2D(self,Array_point,location):
        if Array_point.shape[1] !=2 or location.shape[1] != 2:
            logger.info("不是二维点")
            return
        return np.array([np.sqrt((i[0]-location[0,0])**2 +(i[1]-location[0,1])**2)   for i in Array_point])

    def LOSS(self,Array_point,location,distance):
        location = np.reshape(location,(1,2))
        curdistance = self.Distance2D(Array_point , location)
        #print(curdistance)
        for i in range(len(distance)):
            curdistance[i] = np.sqrt( (curdistance[i]-distance[i])**2 )
        return np.sum(curdistance)
    #梯度下降法
    def SGDS(self,points,distance,learnration=0.01, max_iter_count=300,axis='x',init=[0,0]):
        if points.shape[0] !=3 or distance.shape[0] != 3:
            logger.info("不是三个点")
            return
        loss_global = [0,0,0]    
        eval = np.zeros((3,2))
        for i in range(3):
            iter_list=np.zeros((max_iter_count,2))
            iter_list[0,0] = points[i,0]
            iter_list[0,1] = points[i,1]
            loss_list=np.zeros(max_iter_count)
            iter_count = 0
            loss = 0
            loss_Last=1

            #当迭代此时小于最大迭代次数时，进行
            while  iter_count < max_iter_count-1:        
            #print(str(iter_count)+'  location:'+str(iter_list[iter_count])+ '  loss'+str(loss))
                loss = self.LOSS(points,iter_list[iter_count],distance)
                loss_list[iter_count] = loss
                loss_Last = loss
                iter_count += 1
                if (points[0,0]-iter_list[iter_count-1,0])**2 + (points[0,1]-iter_list[iter_count-1,1])**2 - distance[0]**2>=0:
                    iter_list[iter_count,0] += iter_list[iter_count-1,0] + learnration* (points[0,0]-iter_list[iter_count-1,0])
                    iter_list[iter_count,1] += iter_list[iter_count-1,1] + learnration* (points[0,1]-iter_list[iter_count-1,1])
                else:
                    iter_list[iter_count,0] += iter_list[iter_count-1,0] - learnration* (points[0,0]-iter_list[iter_count-1,0])
                    iter_list[iter_count,1] += iter_list[iter_count-1,1] - learnration* (points[0,1]-iter_list[iter_count-1,1])                
                if (points[1,0]-iter_list[iter_count-1,0])**2 + (points[1,1]-iter_list[iter_count-1,1])**2 - distance[1]**2>=0:
                    iter_list[iter_count,0] +=   learnration* (points[1,0]-iter_list[iter_count-1,0])
                    iter_list[iter_count,1] +=   learnration* (points[1,1]-iter_list[iter_count-1,1])
                else:
                    iter_list[iter_count,0] +=  - learnration* (points[1,0]-iter_list[iter_count-1,0])
                    iter_list[iter_count,1] +=  - learnration* (points[1,1]-iter_list[iter_count-1,1])
                if (points[2,0]-iter_list[iter_count-1,0])**2 + (points[2,1]-iter_list[iter_count-1,1])**2 - distance[2]**2>=0:
                    iter_list[iter_count,0] +=   learnration* (points[2,0]-iter_list[iter_count-1,0])
                    iter_list[iter_count,1] +=   learnration* (points[2,1]-iter_list[iter_count-1,1]) 
                else:
                    iter_list[iter_count,0] +=  - learnration* (points[2,0]-iter_list[iter_count-1,0])
                    iter_list[iter_count,1] +=  - learnration* (points[2,1]-iter_list[iter_count-1,1])             

                loss = self.LOSS(points,iter_list[iter_count],distance)

            minindex = np.argmin(loss_list[0:-1])
            loss_global[i]=(loss_list[minindex])
            eval[i,0] = iter_list[minindex,0]
            eval[i,1] = iter_list[minindex,1]
            #print("迭代位置："+str(minindex))
        minindex = loss_global.index(min(loss_global))
        #print(str(loss_global)+" "+ str(minindex))
        return eval[minindex], loss_global[minindex]    
    #根据uid等索引得到信标坐标
    def Convert(self,lst):
        unpack = []
        for i in lst:
            unpack.extend(i)
        res_dct = {unpack[i]+'_'+unpack[i+1]+'_'+unpack[i+2]: unpack[i+3:i+5] for i in range(0, len(unpack), 5)}
        return res_dct
    def GetXY(self,dct,uid):
        try:
            XY = dct[uid]
            return [float(i) for i in XY]
        except KeyError:
            return -1
    def SumWeightLoc(self,points,distance):    
        if points.shape[0] !=3 or distance.shape[0] != 3:
            logger.info("不是三个点")
            return
        reverse_dis = 1/distance
        weights = reverse_dis/np.sum(reverse_dis)
        return (points[0,:]*weights[0]+points[1,:]*weights[1]+points[2,:]*weights[2])
    # 将step按RSS排序
    def takeRSS(self,elem):
        return elem[3]
    
    #使用三点加权法定位
    #method: SumWeightLoc, SGDSLoc, SGDS_KALMAN_LOC, SGDS_PDR_Loc, SGDS_PDR_KALMAN_Loc
    def Location(self,realtime_json,method='SumWeightLoc'):
        beacon_response,time_seq_beacon ,compass_seq,time_seq_compass, accelerometer_seq,time_seq_accelerometer, deviceMotion_seq, time_seq_deviceMotion,last_loc,last_kalman = self.Load_beacon_seq(realtime_json)
        step = beacon_response
        evalkk, Pkk1,q,r = last_kalman
        evalkk = np.mat(evalkk)
        Pkk1 = np.mat(Pkk1)
        Q = np.mat([[float(q),0],[0,float(q)]])#协方差
        R = np.mat([[float(r),0],[0,float(r)]])#协方差
        cur_step = []
        #logger.info(last_loc)
        for one_beacon in step:   
            
            if self.GetXY(self.beacon_dic,one_beacon[0].lower()+'_'+str(one_beacon[1])+'_'+str(one_beacon[2])) != -1 :
                cur_step.append(one_beacon)
                #print(one_beacon[0].lower()+'_'+str(one_beacon[1])+'_'+str(one_beacon[2]))
            
        if len(cur_step) < 3:
            logger.info('点过少，跳过')
            if method == 'SGDS_PDR_Loc' or method =='SGDS_PDR_KALMAN_Loc':
                x ,y = self.PDR(compass_seq,accelerometer_seq,time_seq_compass,time_seq_accelerometer)
                thita1 , thita2 = self.CalMapTrans()
                res_loc = [last_loc[0]+x*thita1,last_loc[1]+y*thita2]
                return [res_loc, [evalkk.getA().tolist(),Pkk1.getA().tolist(),Q[0,0],R[0,0]]]
            if method == 'SumWeightLoc' or method =='SGDSLoc' or method == 'SGDS_KALMAN_LOC':
                if  len(cur_step) < 1:
                    return [[0,0], [evalkk.getA().tolist(),Pkk1.getA().tolist(),Q[0,0],R[0,0]]]
                else:
                    point = self.GetXY(self.beacon_dic,cur_step[0][0].lower()+'_'+str(cur_step[0][1])+'_'+str(cur_step[0][2]))
                    return [[point[0],point[1]], [evalkk.getA().tolist(),Pkk1.getA().tolist(),Q[0,0],R[0,0]]]
                

        elif len(cur_step) == 3:
            distance = np.zeros(3)
            points = np.ones((3,2))
            distance[0] = cur_step[0][4]
            distance[1] = cur_step[1][4]
            distance[2] = cur_step[2][4]
            points[0] = self.GetXY(self.beacon_dic,cur_step[0][0].lower()+'_'+str(cur_step[0][1])+'_'+str(cur_step[0][2]))
            points[1] = self.GetXY(self.beacon_dic,cur_step[1][0].lower()+'_'+str(cur_step[1][1])+'_'+str(cur_step[1][2]))
            points[2] = self.GetXY(self.beacon_dic,cur_step[2][0].lower()+'_'+str(cur_step[2][1])+'_'+str(cur_step[2][2]))
            if method == 'SumWeightLoc':
                res_loc = self.SumWeightLoc(points,distance)
            if method == 'SGDSLoc':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])
                res_loc = eval
            if method == 'SGDS_KALMAN_LOC':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])
                evalkk1 = evalkk #上一次的卡尔曼估计值为预测值
                Pkk1 = Pkk1 + Q #此次协方差预测值
                Kk = np.dot(Pkk1,(Pkk1+R).I)#最优卡尔曼估计
                evalkk = evalkk1 + np.dot(Kk,np.mat(eval).T - evalkk1)#当前卡尔曼估计值
                Pkk1 = Pkk1 - np.dot(Kk,Pkk1)#当前卡尔曼协方差估计值
                res_loc = ([evalkk[0,0],evalkk[1,0]])  
            if method == 'SGDS_PDR_Loc':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])
                x ,y = self.PDR(compass_seq,accelerometer_seq,time_seq_compass,time_seq_accelerometer)
                thita1 , thita2 = self.CalMapTrans()
                res_loc = ([eval[0]+x*thita1,eval[1]+y*thita2])
            if method == 'SGDS_PDR_KALMAN_Loc':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])
                x ,y = self.PDR(compass_seq,accelerometer_seq,time_seq_compass,time_seq_accelerometer)
                thita1 , thita2 = self.CalMapTrans()
                evalkk1 = evalkk    #上一次的卡尔曼估计值为预测值
                Pkk1 = Pkk1 + Q #此次协方差预测值
                Kk = np.dot(Pkk1,(Pkk1+R).I)#最优卡尔曼估计
                evalkk = evalkk1 + np.dot(Kk,np.mat(eval+[x*thita1,y*thita2]).T - evalkk1)#当前卡尔曼估计值
                Pkk1 = Pkk1 - np.dot(Kk,Pkk1)#当前卡尔曼协方差估计值          
                res_loc = ([eval[0]+x*thita1,eval[1]+y*thita2])                 
        elif len(cur_step) >3:       
            cur_step.sort(key=self.takeRSS,reverse = True)       
            sort3 = cur_step[0:3]
            distance = np.zeros(3)
            points = np.zeros((3,2))
            distance[0] = sort3[0][4]
            distance[1] = sort3[1][4]
            distance[2] = sort3[2][4]
            points[0] = self.GetXY(self.beacon_dic,sort3[0][0].lower()+'_'+str(sort3[0][1])+'_'+str(sort3[0][2]))
            points[1] = self.GetXY(self.beacon_dic,sort3[1][0].lower()+'_'+str(sort3[1][1])+'_'+str(sort3[1][2]))
            points[2] = self.GetXY(self.beacon_dic,sort3[2][0].lower()+'_'+str(sort3[2][1])+'_'+str(sort3[2][2]))
            if method == 'SumWeightLoc':
                res_loc = self.SumWeightLoc(points,distance)
            if method == 'SGDSLoc':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])
                res_loc = eval
            if method == 'SGDS_KALMAN_LOC':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])                 
                evalkk1 = evalkk #上一次的卡尔曼估计值为预测值
                Pkk1 = Pkk1 + Q #此次协方差预测值
                Kk = np.dot(Pkk1,(Pkk1+R).I)#最优卡尔曼估计
                evalkk = evalkk1 + np.dot(Kk,np.mat(eval).T - evalkk1)#当前卡尔曼估计值
                Pkk1 = Pkk1 - np.dot(Kk,Pkk1)#当前卡尔曼协方差估计值
                res_loc = ([evalkk[0,0],evalkk[1,0]])  
            if method == 'SGDS_PDR_Loc':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])
                x ,y = self.PDR(compass_seq,accelerometer_seq,time_seq_compass,time_seq_accelerometer)
                thita1 , thita2 = self.CalMapTrans()
                res_loc = ([eval[0]+x*thita1,eval[1]+y*thita2])
            if method == 'SGDS_PDR_KALMAN_Loc':
                eval, loss = self.SGDS(points,distance,max_iter_count=100,axis='x',init=points[0])
                x ,y = self.PDR(compass_seq,accelerometer_seq,time_seq_compass,time_seq_accelerometer)
                thita1 , thita2 = self.CalMapTrans()
                evalkk1 = evalkk    #上一次的卡尔曼估计值为预测值
                Pkk1 = Pkk1 + Q #此次协方差预测值
                Kk = np.dot(Pkk1,(Pkk1+R).I)#最优卡尔曼估计
                evalkk = evalkk1 + np.dot(Kk,np.mat(eval+[x*thita1,y*thita2]).T - evalkk1)#当前卡尔曼估计值
                Pkk1 = Pkk1 - np.dot(Kk,Pkk1)#当前卡尔曼协方差估计值          
                res_loc = ([eval[0]+x*thita1,eval[1]+y*thita2])                   
        #self.lastpoint =  res_loc
        return res_loc, [evalkk.getA().tolist(),Pkk1.getA().tolist(),Q[0,0],R[0,0]]
    def CalMapTrans(self):
        return np.cos(15/180*np.pi),-np.cos(15/180*np.pi)
    def CalStepLen(self):
        return 0.74*10
    def GetMapDir():
        return np.pi/4
    def PDR(self,compass_data,accelerometer_data,time_compass,time_accelerometer):
        peaks, _ = find_peaks(accelerometer_data, height=0,prominence=0.06,distance=8)
        x = 0
        y = 0
        stepLen = self.CalStepLen()
        seq = 0
        #print(peaks)
        #print(len(time_accelerometer))
        #print('---')
        for index in peaks:
            #print(index)
            timestamp = time_accelerometer[index]  
            for i in range(seq,len(time_compass)):
                if timestamp < time_compass[i]:
                    #print(time_compass[i])
                    seq = i + 1                
                    x = x + stepLen * np.sin(compass_data[i]/180*np.pi)
                    y = y + stepLen * np.cos(compass_data[i]/180*np.pi)                
                    break
                if i == len(time_compass) - 1:
                    x = x + stepLen * np.sin(compass_data[-1]/180*np.pi)
                    y = y + stepLen * np.cos(compass_data[-1]/180*np.pi)                
        return x,y        

    