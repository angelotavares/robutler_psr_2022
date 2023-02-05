import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import locations as loc


class MoveBase:
    def __init__(self, bus):

        self.bus = bus

    def movebase_client(self, g):
       
        cord = loc.locations[g]
        print(cord)
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = cord["x"]
        goal.target_pose.pose.position.y = cord["y"]

        goal.target_pose.pose.orientation.w = cord["w"]
        goal.target_pose.pose.orientation.z = cord["wz"]

        print(goal)
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()
