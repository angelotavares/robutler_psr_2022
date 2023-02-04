from gui_interface import App
from goal_pub import MoveBase
from camera_publisher import ObjectDetection
import threading
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geeteventbus.subscriber import subscriber
from geeteventbus.eventbus import eventbus
from geeteventbus.event import event

classes = open("coco.names").read().strip().split("\n")
np.random.seed(42)
colors = np.random.randint(0, 255, size=(len(classes), 3), dtype="uint8")

# Give the configuration and weight files for the model and load the network.
net = cv2.dnn.readNetFromDarknet("yolov3.cfg", "yolov3.weights")
# self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
# self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Show the network object
# First, get layer names
ln = net.getLayerNames()

# Show names of all layers

# Get output layers
net.getUnconnectedOutLayers()

# Use this layers
try:
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
except IndexError:
    ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()]


class MainProg:
    def __init__(self) -> None:
        self.bus = eventbus()
        self.camera = ObjectDetection(self.bus, net, ln, colors, classes)
        self.goal = MoveBase(self.bus)

        self.gui_sub = gui_events_sub(self)
        self.bus.register_consumer(self.gui_sub, ("gui"))

        self.camera_sub = camera_events_sub(self)
        self.bus.register_consumer(self.camera_sub, ("camera"))

        rospy.init_node("object_detection")

        self.gui = App(self.bus)

    def brains(self, topic, data):
        if topic == "gui":
            print("data", data)
        else:
            print(data)


class gui_events_sub(subscriber):
    def __init__(self, mp):
        self.mp = mp

    def process(self, eventobj):
        if not isinstance(eventobj, event):
            print("Invalid object type is passed.")
            return

        topic = eventobj.get_topic()
        data = eventobj.get_data()

        if topic == "gui":
            self.mp.brains(topic, data)


class camera_events_sub(subscriber):
    def __init__(self, mp):
        self.mp = mp

    def process(self, eventobj):
        if not isinstance(eventobj, event):
            print("Invalid object type is passed.")
            return

        topic = eventobj.get_topic()
        data = eventobj.get_data()

        if topic == "camera":
            self.mp.brains(topic, data)


main_prop = MainProg()

while True:
    pass
