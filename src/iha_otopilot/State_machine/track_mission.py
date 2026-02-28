#!/usr/bin/env python3
import time
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleGlobalPosition
import math


"""

"""

class State(Enum):
    BASLANGIC = 0       
    KALKIS = 1          
    SEYIR = 2           
    ARAMA = 3           
    TAKIP = 4          
    GOREV = 5           
    EVE_DONUS = 6       
    ACIL_INIS = 112

class track_mission():
    def __init__(self):
        
        self.suanki_durum = State.BASLANGIC
        self.hedef_irtifa = 5.0 
        self.gorev_yapildi_mi = False 