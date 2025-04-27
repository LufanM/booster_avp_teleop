# subscriber.py

from avp_teleop import VisionProStreamer
import argparse 
from typing import * 
import time

if __name__ == "__main__": 

    parser = argparse.ArgumentParser()
    # parser.add_argument('--ip', type = str, required = True)
    # parser.add_argument('--record', action = 'store_true')
    # args = parser.parse_args()

    avp_ip = "192.168.101.131"   # example IP 
    s = VisionProStreamer(ip = avp_ip, record = True)
    s.start_streaming()

    start_time = time.time()  # 记录开始时间
    duration = 10  # 运行持续时间（秒）

    while time.time() - start_time < duration:
        latest = s.latest
        print("时间：", time.time() - start_time)
        print("头部数据：", latest['head'][:, :, :])
        print("右手数据：", latest['right_wrist'][:, :, :])
        print("右手手指数据：", latest['right_fingers'].shape)
        time.sleep(0.5)

    s.stop_streaming()

    
    s.start_streaming()
    while (time.time() - start_time >= 15) and (time.time() - start_time < 20):
        latest = s.latest
        print("头部数据：", latest['head'][:, :, :])
        print("右手数据：", latest['right_wrist'][:, :, :])
        print("右手手指数据：", latest['right_fingers'].shape)
        time.sleep(0.5)