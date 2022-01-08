from django.urls import path,include
from . import views, updateMap

urlpatterns = [
    path('localization/',views.Localization,name="localization"),
    path('updateMap/',updateMap.updateMap,name="updateMap"),
]
