from django.shortcuts import render
from django.http import HttpResponse,JsonResponse   ##JsonResponse返回json数据
from django.views.decorators.csrf import csrf_exempt
from . import apps,models
from BLE_API.settings import BASE_DIR
import os
from django.template import loader
from django.core.files.base import ContentFile
from datetime import date

@csrf_exempt
def updateMap(request):    
    if request.method == 'GET':    
        template = loader.get_template('updateMap.html')
        context = {
        }
        return HttpResponse(template.render(context,request))
    elif request.method == "POST":
        mapID = request.POST.get('mapID')
        obj = request.FILES.get('file')
        handle_uploaded_file(mapID,ContentFile(obj.read()))
        return HttpResponse('上传成功')
        
def handle_uploaded_file(name,f):
    #with open(name+'.txt', 'wb') as destination:
    #    for chunk in f.chunks():
    #        destination.write(chunk)
    #        print(chunk)
    a = ""
    #print(1)
    for chunk in f.chunks():
        a = a+ chunk.decode("utf-8")
    b = a.split("\n")
    for line in b:
        if len(line) >0:
            mapID = name
            tmp = line.split(',')
            beaconID = tmp[0]+'_'+tmp[1]+'_'+tmp[2]
            x = float(tmp[3])
            y = float(tmp[4] )   
            today = date.today()
            if  models.Map.objects.filter(mapID=mapID, beaconID=beaconID, x=x, y=y).exists():
                models.Map.objects.filter(mapID=mapID, beaconID=beaconID, x=x, y=y).delete()
            map = models.Map.objects.create(mapID=mapID, beaconID=beaconID, x=x, y=y,load_date = today.strftime('%Y-%m-%d'))
            print(map, type(map))
    