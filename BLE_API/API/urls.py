from django.urls import path,include
from . import views

urlpatterns = [
    path('localization/',views.Localization,name="localization"),
]
