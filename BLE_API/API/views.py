from django.shortcuts import render
from django.http import HttpResponse,JsonResponse   ##JsonResponse返回json数据
from django.views.decorators.csrf import csrf_exempt
from . import apps
from BLE_API.settings import BASE_DIR
import os
import json
import logging
logger =  logging.getLogger('Request')
# Create your views here.
"""
简单body请求报文
{
    "x":int,
    "y":int,
}
"""
##简单定义返回报文
###evalkk 当前卡尔曼估计值
###Pkk1 当前卡尔曼协方差估计值
###Q R 观察与状态数据、协方差
res =  {
        "location":{
        "x": 0,
        "y": 0},
        "kalmanFactor":{
        "evalkk":[0,0],
        "Pkk1":0,
        "Q":0,
        "R":0,
        },
        "msg":"",
        "status":"",
}

##发送的Request格式
###[baecon_json, compass_json,加速器json，设备方向json，上次位置json,卡尔曼滤波参数json]
###[baecon_json, compass_json,加速器json，设备方向json，last_loc,last_kalman]
###{"res":[x,y],"timestamp":1632981363.257,"type":"last_loc"}
###{"res":[evalkk,Pkk1,Q,R],"timestamp":1632981363.257,"type":"last_kalman"}
Location = apps.ThreePointLoc(os.path.join(BASE_DIR,'API',"蓝牙信标数据（完善）.txt"))
@csrf_exempt
def Localization(request):
    if request.method == 'POST':
        ##判断POST请求body是否为空
        
        if request.body.decode("utf-8") == '':
            res['status'] = "Error"
            res['msg'] = "body is Null!"
            logger.error("body is Null!")
            return JsonResponse(res)
        ##不为空就将body转换成字典
        body = request.body.decode("utf-8")
        ##确保字段不为空
        res['status'] = "Success"
        eval,kalmanFactor = Location.Location(body,method='SGDSLoc')
        res['location']['x'] = eval[0]
        res['location']['y'] = eval[1]
        res['kalmanFactor']['evalkk'] = kalmanFactor[0]
        res['kalmanFactor']['Pkk1'] = kalmanFactor[1]
        res['kalmanFactor']['Q'] = kalmanFactor[2]
        res['kalmanFactor']['R'] = kalmanFactor[3]
        #logger.error(res)
        return JsonResponse( res)
    else:
        res['status'] = "Error"
        res['msg'] = "request method not is POST!"
        return JsonResponse(res)
