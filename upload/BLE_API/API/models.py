from django.db import models

# Create your models here.
#每个用户的每次定位的参数
class Customer(models.Model):
    name = models.CharField(max_length=30)
    last_loc_x = models.DecimalField(max_digits=10, decimal_places=2)
    last_loc_y = models.DecimalField(max_digits=10, decimal_places=2)
    last_kal_p1 = models.DecimalField(max_digits=10, decimal_places=2)
    last_kal_p2 = models.DecimalField(max_digits=10, decimal_places=2)
    last_kal_p3 = models.DecimalField(max_digits=10, decimal_places=2)
    last_kal_p4 = models.DecimalField(max_digits=10, decimal_places=2)
    last_kal_Q = models.DecimalField(max_digits=10, decimal_places=2)
    last_kal_R = models.DecimalField(max_digits=10, decimal_places=2)
    customermap = models.ManyToManyField("CustomerMap")
    require_date = models.DateField()
    mapID = models.OneToOneField("Map", on_delete=models.CASCADE)


#每个用户每次的RSSI滤波参数    
class CustomerMap(models.Model):
    name = models.CharField(max_length=30)
    mapID = models.OneToOneField("Map", on_delete=models.CASCADE)
    last_RSSI = models.DecimalField(max_digits=10, decimal_places=2)
    last_Q = models.DecimalField(max_digits=10, decimal_places=2)
    last_R = models.DecimalField(max_digits=10, decimal_places=2)
    require_date = models.DateField()


class Map(models.Model):
    mapID = models.CharField(max_length=32)
    beaconID = models.CharField(max_length=50,default='')
    x = models.DecimalField(max_digits=10, decimal_places=2)
    y = models.DecimalField(max_digits=10, decimal_places=2)
    load_date = models.DateField()
 
#在管理员站点加入地图操作功能 
class MapOpe(models.Model):
    pass