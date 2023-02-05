from gui_interface import App
from goal_pub import MoveBase
from camera_publisher import ObjectDetection
from vel_pub import VelPub
from speech_iteraction import TalkTO
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
        self.vel_pub = VelPub()
        self.talk = TalkTO()
        self.gui_sub = gui_events_sub(self)
        self.bus.register_consumer(self.gui_sub, ("gui"))

        self.camera_sub = camera_events_sub(self)
        self.bus.register_consumer(self.camera_sub, ("camera"))

        rospy.init_node("object_detection")

        self.gui = App(self.bus)

        self.front_vel = 0.5
        self.rotate_vel = 0.5
        self.object_present = ""

    def check_if_objective(self, objective):
        actions = [
            "Move",
            "Look for sphere",
            "Is there anyone",
            "Is Suitcase There",
            "Is the table clean",
        ]
        if objective == actions[0]:
            "Arrive at Location"
        elif objective == actions[1] and (self.object_present == "sphere" or self.object_present == "ball") :
            print("There is a sphere There")
        elif objective == actions[2] and self.object_present == "person":
            print("Persopn present")

        elif objective == actions[3] and self.object_present == "suitcase":
            print("Yes")
        elif objective == actions[4] and self.object_present == "":
            print("Tabler clean")
        else:
            print("No objective")
        

    def brains(self, topic, data):
        if topic == "gui":
            print("data", data)

            if data.split(":")[0] == "move":
                if data.split(":")[1] == "Forward":
                    print(type(self.front_vel))
                    self.vel_pub.robutler_move(lx=self.front_vel)
                elif data.split(":")[1] == "Backwards":
                    self.vel_pub.robutler_move(lx=-self.front_vel)
                elif data.split(":")[1] == "Rotate Left":
                    self.vel_pub.robutler_move(az=self.rotate_vel)
                elif data.split(":")[1] == "Rotate Right":
                    self.vel_pub.robutler_move(az=-self.rotate_vel)
                else:
                    self.vel_pub.robutler_move()
            elif data.split(":")[0] == "vel":
                self.front_vel = float(data.split(":")[1])
            elif data.split(":")[0] == "velh":
                self.rotate_vel = float(data.split(":")[1])
            elif data.split(":")[0] == "talking":
                self.talk.talking_to()
            else:
                room = data.split(":")[1]
                action = data.split(":")[0]
                sucess = self.goal.movebase_client(room)
                print(sucess)
                self.check_if_objective(action)
        elif topic == "camera":
            print(data)
            self.object_present = data


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
