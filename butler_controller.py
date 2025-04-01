#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from butler.msg import TableStatus

class ButlerController:
    def __init__(self):
        rospy.init_node('butler_controller')
        
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalStatusArray, queue_size=10)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.Subscriber('/table_confirm', Bool, self.table_confirm_callback)
        rospy.Subscriber('/kitchen_confirm', Bool, self.kitchen_confirm_callback)
        rospy.Subscriber('/table1', TableStatus, self.table1_callback)
        rospy.Subscriber('/table2', TableStatus, self.table2_callback)
        rospy.Subscriber('/table3', TableStatus, self.table3_callback)
        
        self.reached_goal = False
        self.table_confirm = False
        self.kitchen_confirm = False
        self.table_orders = {"table1": {'order': False, 'cancel': False}, "table2": {'order': False, 'cancel': False}, "table3": {'order': False, 'cancel': False}}
        
        self.locations = {
            "kitchen": PoseStamped(),
            "table1": PoseStamped(),
            "table2": PoseStamped(),
            "table3": PoseStamped(),
            "home": PoseStamped()
        }
        
        self.set_location_coordinates()

    def set_location_coordinates(self):
        """Initialize the predefined locations."""
        coords = {
            "kitchen": (-1.5, -5.1, 0, 0, 0, 0.6, 0.72),
            "table1": (-1.5, -1.7, 0, 0, 0, 0.6, 0.7),
            "table2": (0.6, 5.9, 0, 0, 0, 0.2, 0.9),
            "table3": (3.5, 7.0, 0, 0, 0, 0.99, 0),
            "home": (1.5, -3.5, 0, 0, 0, 0.9, 0.3)
        }
        for key, (x, y, z, ox, oy, oz, ow) in coords.items():
            self.locations[key].header.frame_id = "map"
            self.locations[key].pose.position.x = x
            self.locations[key].pose.position.y = y
            self.locations[key].pose.position.z = z
            self.locations[key].pose.orientation.x = ox
            self.locations[key].pose.orientation.y = oy
            self.locations[key].pose.orientation.z = oz
            self.locations[key].pose.orientation.w = ow

    def move_to_location(self, location):
        """Publish the goal to move_base_simple/goal."""
        if location in self.locations:
            rospy.loginfo(f"Moving to {location}")
            self.goal_pub.publish(self.locations[location])
            self.reached_goal = False
        else:
            rospy.logwarn("Invalid location request!")

    def cancel_current_goal(self):
        """Cancel the current goal."""
        rospy.loginfo("Cancelling current goal.")
        self.cancel_pub.publish(GoalStatusArray())

    def status_callback(self, msg):
        """Check if the robot has reached the goal."""
        if msg.status_list:
            latest_status = msg.status_list[-1].status
            if latest_status == 3:  # Goal Succeeded
                self.reached_goal = True
                #rospy.loginfo("✅ Goal Reached!")
            else:
                self.reached_goal = False
                #rospy.loginfo("Goal not reached !")

    def table_confirm_callback(self, msg):
        """Receive confirmation from the table."""
        self.table_confirm = msg.data

    def kitchen_confirm_callback(self, msg):
        """Receive confirmation from the kitchen."""
        self.kitchen_confirm = msg.data

    def table1_callback(self, msg):
        """Update order status for table1."""
        self.table_orders['table1']['order'] = msg.order
        self.table_orders['table1']['cancel'] = msg.cancel

    def table2_callback(self, msg):
        """Update order status for table2."""
        self.table_orders['table2']['order'] = msg.order
        self.table_orders['table2']['cancel'] = msg.cancel

    def table3_callback(self, msg):
        """Update order status for table3."""
        self.table_orders['table3']['order'] = msg.order
        self.table_orders['table3']['cancel'] = msg.cancel

    def execute_delivery(self):
        """Main loop for executing the butler sequence."""
        tables = ["table1", "table2", "table3"]
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            valid_orders = [table for table in tables if self.table_orders[table]['order'] and not self.table_orders[table]['cancel']]
            
            if not valid_orders:
                rate.sleep()
                continue
            
            self.move_to_location("kitchen")
            while not self.reached_goal and not rospy.is_shutdown():
                rate.sleep()
            
            rospy.loginfo("Waiting for kitchen confirmation...")
            timeout = rospy.Time.now() + rospy.Duration(10)
            while not self.kitchen_confirm and rospy.Time.now() < timeout and not rospy.is_shutdown():
                rate.sleep()
            if not self.kitchen_confirm:
                rospy.loginfo("Kitchen confirmation not received. Returning home.")
                self.move_to_location("home")
                while not self.reached_goal and not rospy.is_shutdown():
                    rate.sleep()
                continue
            self.kitchen_confirm = False
            
            for table in tables:
                valid_orders = [t for t in tables if self.table_orders[t]['order'] and not self.table_orders[t]['cancel']]
                if table not in valid_orders:
                    continue
                
                self.move_to_location(table)
                while not self.reached_goal and not rospy.is_shutdown():
                    if self.table_orders[table]['cancel']:
                        rospy.loginfo(f"Cancelling current goal and skipping {table}.")
                        self.cancel_current_goal()
                        break
                    rate.sleep()
                
                if not self.table_orders[table]['cancel']:
                    rospy.loginfo(f"Waiting for confirmation at {table}...")
                    timeout = rospy.Time.now() + rospy.Duration(10)
                    while not self.table_confirm and rospy.Time.now() < timeout and not rospy.is_shutdown():
                        rate.sleep()
                    
                self.table_confirm = False
                #self.table_orders[table]['order'] = False
                
            if any(self.table_orders[table]['order'] and self.table_orders[table]['cancel'] for table in self.table_orders):
                    self.move_to_location("kitchen")
                    while not self.reached_goal and not rospy.is_shutdown():
                        rate.sleep()
                        
            self.move_to_location("home")
            while not self.reached_goal and not rospy.is_shutdown():
                rate.sleep()

if __name__ == '__main__':
    
    controller = ButlerController()
    controller.execute_delivery()
