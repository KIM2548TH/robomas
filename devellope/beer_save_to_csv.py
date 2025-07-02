# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import robomaster
from robomaster import robot
import pandas as pd
import datetime
import os

# Lists to store data
position_data = []
attitude_data = []
imu_data = []
esc_data = []
status_data = []

#1
def sub_info_position(sub_info):
    timestamp = datetime.datetime.now()
    print("Position sub info: {0}".format(sub_info))
    position_data.append({
        'timestamp': timestamp,
        'data': str(sub_info)
    })

#2
def sub_info_attitude(sub_info):
    timestamp = datetime.datetime.now()
    print("Attitude sub info: {0}".format(sub_info))
    attitude_data.append({
        'timestamp': timestamp,
        'data': str(sub_info)
    })

#3
def sub_info_imu(sub_info):
    timestamp = datetime.datetime.now()
    print("IMU sub info: {0}".format(sub_info))
    imu_data.append({
        'timestamp': timestamp,
        'data': str(sub_info)
    })

#4
def sub_info_esc(sub_info):
    timestamp = datetime.datetime.now()
    print("ESC sub info: {0}".format(sub_info))
    esc_data.append({
        'timestamp': timestamp,
        'data': str(sub_info)
    })

#5
def sub_info_status(sub_info):
    timestamp = datetime.datetime.now()
    print("Status sub info: {0}".format(sub_info))
    status_data.append({
        'timestamp': timestamp,
        'data': str(sub_info)
    })

def save_data_to_csv():
    """Save all collected data to CSV files in csv folder"""
    # Create csv directory if it doesn't exist
    csv_dir = "csv"
    if not os.path.exists(csv_dir):
        os.makedirs(csv_dir)
    
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    if position_data:
        df_position = pd.DataFrame(position_data)
        filepath = os.path.join(csv_dir, f'position_data_{timestamp_str}.csv')
        df_position.to_csv(filepath, index=False)
        print(f"Position data saved to {filepath}")
    
    if attitude_data:
        df_attitude = pd.DataFrame(attitude_data)
        filepath = os.path.join(csv_dir, f'attitude_data_{timestamp_str}.csv')
        df_attitude.to_csv(filepath, index=False)
        print(f"Attitude data saved to {filepath}")
    
    if imu_data:
        df_imu = pd.DataFrame(imu_data)
        filepath = os.path.join(csv_dir, f'imu_data_{timestamp_str}.csv')
        df_imu.to_csv(filepath, index=False)
        print(f"IMU data saved to {filepath}")
    
    if esc_data:
        df_esc = pd.DataFrame(esc_data)
        filepath = os.path.join(csv_dir, f'esc_data_{timestamp_str}.csv')
        df_esc.to_csv(filepath, index=False)
        print(f"ESC data saved to {filepath}")
    
    if status_data:
        df_status = pd.DataFrame(status_data)
        filepath = os.path.join(csv_dir, f'status_data_{timestamp_str}.csv')
        df_status.to_csv(filepath, index=False)
        print(f"Status data saved to {filepath}")

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # สมัครรับข้อมูลตำแหน่งของแชสซี
    ep_chassis.sub_position(freq=1, callback=sub_info_position)

    # สมัครรับข้อมูลท่าทางของแชสซี
    ep_chassis.sub_attitude(freq=5, callback=sub_info_attitude)

    # สมัครรับข้อมูล IMU ของแชสซี
    ep_chassis.sub_imu(freq=10, callback=sub_info_imu)

    # สมัครรับข้อมูลตัวควบคุมมอเตอร์ของแชสซี
    ep_chassis.sub_esc(freq=20, callback=sub_info_esc)

    # สมัครรับข้อมูลสถานะของแชสซี:
    ep_chassis.sub_status(freq=50, callback=sub_info_status)

    time.sleep(10)

    ep_chassis.unsub_status()
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()

    # Save all collected data to CSV files
    save_data_to_csv()

    ep_robot.close()