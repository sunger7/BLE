from django.contrib import admin
from django.shortcuts import render

from .updateMap import updateMap

# Register your models here.
from .models import Map,MapOpe


class MapAdmin(admin.ModelAdmin):
    list_display = ['mapID', 'beaconID','x','y','load_date']
    ordering = ['mapID']
    
    
admin.site.register(Map,MapAdmin)
@admin.register(MapOpe)
class MapOpeAdmin(admin.ModelAdmin):
    def changelist_view(self, request, extra_content=None):
        return updateMap(request)