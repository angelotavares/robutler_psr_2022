import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


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


class ObjectDetection:
    def __init__(self, net, ln, colors, classes):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw_arm",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**32,
        )
        self.image_pub = rospy.Publisher("/cam_processed/output", Image, queue_size=1)

        self.net = net
        self.ln = ln
        self.colors = colors
        self.classes = classes

    def image_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Load names of classes and get random colors for them.

        # Construct a blob from the image

        blob = cv2.dnn.blobFromImage(
            cv_image, 1 / 255.0, (416, 416), swapRB=True, crop=False
        )
        r = blob[0, 0, :, :]

        self.net.setInput(blob)

        outputs = self.net.forward(self.ln)

        boxes = []
        confidences = []
        classIDs = []
        h, w = cv_image.shape[:2]

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                if confidence > 0.5:
                    box = detection[:4] * np.array([w, h, w, h])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    box = [x, y, int(width), int(height)]
                    boxes.append(box)
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        if len(indices) > 0:
            for i in indices.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = [int(c) for c in self.colors[classIDs[i]]]

                #  -- Arguments for CV2 rectangle:
                # cv2.rect   (cv_image,  x, y,   width, height, color, line width)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 4)

                # Labels and confidences for the image
                text = "{}: {:.4f}".format(self.classes[classIDs[i]], confidences[i])
                cv2.putText(
                    cv_image,
                    text,
                    (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )

                # Convert the image back to a ROS message and publish it on the output topic
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node("object_detection")
    ir = ObjectDetection(net, ln, colors, classes)
    rospy.spin()
