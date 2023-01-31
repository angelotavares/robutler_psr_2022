import sys
from geeteventbus.subscriber import subscriber
from geeteventbus.eventbus import eventbus
from geeteventbus.event import event

sys.path.append("..")

from robutler_camera.src.camera_publisher import ObjectDetection

class Robutler:

    def __init__(self) -> None:
        

        self.bus = eventbus()
        
        
        self.camera_sub = camera_node(self)
        self.bus.register_consumer(self.camera_sub, 'object_detected')
        
        self.object_detected = []
        
        
    
class camera_node(subscriber):
    """
    [extended_summary]

    Args:
        subscriber ([type]): [description]
    """

    def __init__(self, ct):
        self.ct = ct

    def process(self, eventobj):
        if not isinstance(eventobj, event):
            print('Invalid object type is passed.')
            return

        topic = eventobj.get_topic()
        data = eventobj.get_data()
        self.ct.objected_detected.append(data)