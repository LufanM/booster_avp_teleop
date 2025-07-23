import grpc
from avp_teleop.grpc_msg import *
from threading import Thread
from avp_teleop.utils.grpc_utils import *
import numpy as np
from termcolor import cprint


YUP2ZUP = np.array([[[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]], dtype=np.float64)


class VisionProStreamer:
    def __init__(self, ip, record=True):

        # Vision Pro IP
        self.ip = ip
        self.record = record
        self.recording = []
        self.latest = None
        self.axis_transform = np.eye(4)
        self.failed_to_start = False

    def start_streaming(self):

        self.stream_thread = Thread(target=self.stream)
        self.stream_thread.start()
        while self.latest is None:
            if self.failed_to_start:
                cprint("Failed to start streaming. Please check the IP address.", "red")
                raise RuntimeError("Failed to start streaming.")
            pass
        print(" == DATA IS FLOWING IN! ==")
        print("Ready to start streaming.")

    def stop_streaming(self):
        # 等待线程结束
        if self.stream_thread is not None:
            self.stream_thread.join()
            
        self.latest = None
        self.failed_to_start = False
        self.stream_thread = None
        print(" == DATA FLOWING STOP! ==")

    def stream(self):
        request = handtracking_pb2.HandUpdate()
        try:
            with grpc.insecure_channel(f"{self.ip}:12345") as channel:
                stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                responses = stub.StreamHandUpdates(request)
                for response in responses:
                    # print(f"receive info : {response.left_hand.wristMatrix}")
                    transformations = {
                        "left_wrist": self.axis_transform @ process_matrix(response.left_hand.wristMatrix),
                        "right_wrist": self.axis_transform @ process_matrix(response.right_hand.wristMatrix),
                        "left_fingers": process_matrices(response.left_hand.skeleton.jointMatrices),
                        "right_fingers": process_matrices(response.right_hand.skeleton.jointMatrices),
                        "head": rotate_head(self.axis_transform @ process_matrix(response.Head)),
                        "left_pinch_distance": get_pinch_distance(response.left_hand.skeleton.jointMatrices),
                        "right_pinch_distance": get_pinch_distance(response.right_hand.skeleton.jointMatrices),
                        "left_pinch_distance_middle": get_pinch_distance2(response.left_hand.skeleton.jointMatrices),
                        "right_pinch_distance_middle": get_pinch_distance2(response.right_hand.skeleton.jointMatrices),
                        # "rgb": response.rgb, # TODO: should figure out how to get the rgb image from vision pro and elbow data
                    }
                    transformations["right_wrist_roll"] = get_wrist_roll(
                        transformations["right_wrist"]
                    )
                    transformations["left_wrist_roll"] = get_wrist_roll(
                        transformations["left_wrist"]
                    )
                    if self.record:
                        self.recording.append(transformations)
                    self.latest = transformations

        except Exception as e:
            self.failed_to_start = True
            print(f"An error occurred: {e}")
            return

    def get_latest(self):
        return self.latest

    def get_recording(self):
        return self.recording


if __name__ == "__main__":

    streamer = VisionProStreamer(ip="10.29.230.57")
    while True:

        latest = streamer.get_latest()
        print(latest)
