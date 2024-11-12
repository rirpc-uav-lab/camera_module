import rclpy
from rclpy.node import Node

import requests

import time
import sys
import numpy as np
import math
from copy import copy

from std_msgs.msg import String

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from uav_msgs.srv import Micasense

import platform


class MicasenseCommands(Node):

    def __init__(self, username=None):
        
        super().__init__('micasense_commands')

        self.diagnostic_pub = self.create_publisher(DiagnosticStatus, "uav1/micasense", 10)

        self.srv = self.create_service(Micasense, 'commands_service', self.commands_callback)
       
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)



    def commands_callback(self, request, response):
        
        # Одиночное фото

        if request.cmd == 1:
            auto_cap_flag = requests.get('http://192.168.1.83/status')
            if auto_cap_flag.json()['auto_cap_active'] == False: 

                capture_params = { 'store_capture' : True, 'block' : True }
                capture_data = requests.get("http://192.168.10.254/capture", json=capture_params)

                status = capture_data.json()['status']

                if status == "complete":
                    response.result = 4
        
                else:
                    response.result = 5
            
            else:
                response.result = 5


        # Съёмка по периоду
        else:
            
            if request.cmd == 2:

                auto_cap_flag = requests.get('http://192.168.1.83/status')
                if auto_cap_flag.json()['auto_cap_active'] == False:

                    time_period = request.time_period # не путать request и requests

                    config_params = { 'auto_cap_mode' : "timer", 'timer_period' : time_period }
                    config_data = requests.post("http://192.168.10.254/config", json=config_params)

                    capture_params = { 'store_capture' : True, 'block' : False }
                    capture_data = requests.post("http://192.168.10.254/capture", json=capture_params)

                    auto_cap_flag = requests.get('http://192.168.1.83/status')
                    if auto_cap_flag.json()['auto_cap_active'] == True:
                        response.result = 4
                    else:
                        response.result = 5

                else:
                    response.result = 5
        
        #Остановить съёмку
            else:
                if request.cmd == 3:
                    auto_cap_flag = requests.get('http://192.168.1.83/status')
                    if auto_cap_flag.json()['auto_cap_active'] == True:
                    
                        config_params = { 'auto_cap_mode' : "disabled" }
                        config_data = requests.post("http://192.168.10.254/config", json=config_params)

                        auto_cap_flag = requests.get('http://192.168.1.83/status')
                        if auto_cap_flag.json()['auto_cap_active'] == False:
                            response.result = 4
                        else:
                            response.result = 5
                    
                    else:
                        response.result = 5
                
                else:
                    response.result = 5
                    

        return response




    def timer_callback(self):
        
        diagnost_msg = DiagnosticStatus()
        keyvalue = KeyValue()

        memory_data = requests.get('http://192.168.1.83/networkstatus')
        free_value = str(gps_data.json()['network_map'][0]['sd_gb_free'])
        total_value = str(gps_data.json()['network_map'][0]['sd_gb_total'])

        diagnost_msg.hardware_id = "uav1/micasense"

        keyvalue.key = "remaining_storage"
        keyvalue.value = free_value
        
        free_space = float(free_value)
        total_space = float(total_value)

        if free_space/total_space > 0.1:
            diagnost_msg.level = DiagnosticStatus.OK
        else:
            diagnost_msg.level = DiagnosticStatus.WARNING
            diagnost_msg.message = "свободного места на sd карте менее 10 % (" + free_value + " Гб)"
        
        diagnost_msg.values.append(keyvalue)

        self.diagnostic_pub.publish(diagnost_msg)


            
def main(args=None):
    rclpy.init(args=args)

    micasense_commands = None

    micasense_commands = MicasenseCommands()

    rclpy.spin(micasense_commands)

    
    micasense_commands.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






