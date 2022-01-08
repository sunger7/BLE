from django.apps import AppConfig
import numpy as np
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
        evalkk=np.mat([[1637],[1219]])#当前卡尔曼估计值
        evalkk1=np.mat([[1628],[1211]])#当前预测值
        self.evalk1k1=np.mat([[0],[0]])#上一次卡尔曼估计值
        self.Pkk=np.mat([[100,0],[0,100]])
        self.Pkk1=np.mat([[100,0],[0,100]])
        self.Pk1k1=np.mat([[100,0],[0,100]])
        self.Q = np.mat([[100,0],[0,100]])
        self.R = np.mat([[200,0],[0,200]])
        self.lastpoint = [0,0]
        
    def Load_beacon_seq(self,realtime_json):
        
        res_sparse = json.loads(realtime_json)
        #logger.error((res_sparse[0:2]))
        beacon_response = []
        compass_samples = []
        accelerometer_samples = []
        deviceMotion_samples = []
        for res in res_sparse:
            #解析信标
            if res['type'] == 'ibeacon':
                for one_beacon in res['res']:
                    beacon_response.append(one_beacon)
            if res['type'] == 'compass':
                #解析罗盘
                compass_samples.append(res['res'])
            if res['type'] == 'accelerometer':
                #解析加速器
                accelerometer_samples.append(res['res'])
            if res['type'] == 'deviceMotion':
                #解析设备方向
                deviceMotion_samples.append(res['res'])
        return beacon_response, compass_samples, accelerometer_samples, deviceMotion_samples
        
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
    #method: SumWeightLoc, SGDSLoc, SGDS_KALMAN_LOC
    def Location(self,realtime_json,method='SumWeightLoc'):
        beacon_response, compass_samples, accelerometer_samples, deviceMotion_samples = self.Load_beacon_seq(realtime_json)
        step = beacon_response
        #print(len(beacon_response))
        cur_step = []
        for one_beacon in step:        
            if self.GetXY(self.beacon_dic,one_beacon[0].lower()+'_'+str(one_beacon[1])+'_'+str(one_beacon[2])) != -1 :
                cur_step.append(one_beacon)
                #print(one_beacon[0].lower()+'_'+str(one_beacon[1])+'_'+str(one_beacon[2]))
            
        if len(cur_step) < 3:
            logger.info('点过少，跳过')
            return self.lastpoint

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
                eval, loss = self.SGDS(points,distance)
                res_loc = eval
            if method == 'SGDS_KALMAN_LOC':
                eval, loss = self.SGDS(points,distance)
                res_loc = eval                
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
                eval, loss = self.SGDS(points,distance)
                res_loc = eval
            if method == 'SGDS_KALMAN_LOC':
                eval, loss = self.SGDS(points,distance)                 
                self.evalkk1 = self.evalkk #上一次的卡尔曼估计值为预测值
                self.Pkk1 = self.Pk1k1 + self.Q #此次协方差预测值
                Kk = np.dot(self.Pkk1,(self.Pkk1+self.R).I)#最优卡尔曼估计
                self.evalkk = self.evalkk1 + np.dot(Kk,np.mat(eval).T - self.evalkk1)#当前卡尔曼估计值
                self.Pkk1 = self.Pkk1 - np.dot(self.Kk,self.Pkk1)#当前卡尔曼协方差估计值
                res_loc = ([self.evalkk[0,0],self.evalkk[1,0]])             
        self.lastpoint =  res_loc
        return res_loc
        

    