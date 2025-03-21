import pandas as pd
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from openpyxl import Workbook
from datetime import datetime




class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')

        topic_name = '/robot01/odom'
        #change /robot01 with the name of the robot kept



        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odom_callback,
            qos_profile
            )
        self.subscription  # prevent unused variable warning
        self.workbook = Workbook()
        self.position_worksheet = self.workbook.active
        self.position_worksheet.title = 'Position data '
        self.position_worksheet.append(["Timestamp", "odo_poss_X", "odo_poss_Y","odo_poss_Z"])




        self.orientation_worksheet = self.workbook.create_sheet(title= 'Orientation data ')
        self.orientation_worksheet.append(["Timestamp", "odo_orient_X", "odo_orient_Y","odo_orient_Z","odo_orient_W"])

        self.lineartwist_worksheet = self.workbook.create_sheet(title= 'Linear twist data ')
        self.lineartwist_worksheet.append(["Timestamp", "odo_lintw_X", "odo_lintw_Y","odo_lintw_Z"])


        self.angulartwist_worksheet = self.workbook.create_sheet(title= 'Angular twist data ')
        self.angulartwist_worksheet.append(["Timestamp", "odo_angtw_X", "odo_angtw_Y","odo_angtw_Z"])

        # 4 seperate sheets are created for position, orientation, linear twist and angular twist data for clarity


        self.position_data = []
        self.orientation_data = []
        self.lineartwist_data = []
        self.angulartwist_data = []
         #all the data which will be added in the excel sheets, are first added in these list, then once the node is terminated
        # all the data from the list will be then saved in the excel file.


        
        self.timer = self.create_timer(1.0, self.process_buffered_messages)
        # in gap of 1 seconds the function process_buffered_messages will be called, so that the latest message from the topic which
        # was stored in the lists self.message_buffer can be extracted and stored in the data list.
        
        
        self.message_buffer = []  # a list is created so that all the readings from the topic can be stored in it.




    def odom_callback(self, msg):
        self.message_buffer.append(msg)


    def process_buffered_messages(self):
        if not self.message_buffer:
            return
        
        latest_msg = self.message_buffer[-1]
        #self.get_logger().info(str(msg))
        pos_x = latest_msg.pose.pose.position.x
        pos_y = latest_msg.pose.pose.position.y
        pos_z = latest_msg.pose.pose.position.z

        orient_x = latest_msg.pose.pose.orientation.x
        orient_y = latest_msg.pose.pose.orientation.y
        orient_z = latest_msg.pose.pose.orientation.z
        orient_w = latest_msg.pose.pose.orientation.w

        litwist_x = latest_msg.twist.twist.linear.x
        litwist_y = latest_msg.twist.twist.linear.y
        litwist_z = latest_msg.twist.twist.linear.z

        angtwist_x = latest_msg.twist.twist.angular.x
        angtwist_y = latest_msg.twist.twist.angular.y
        angtwist_z = latest_msg.twist.twist.angular.z

        self.get_logger().info(f"Odometer data: \n - Position data --> x: {pos_x} y: {pos_y} z: {pos_z}\n - Orientation data-->  x:{orient_x} y: {orient_y} z: {orient_z} w:{orient_w} \n - Linear twist data -->x: {litwist_x} y: {litwist_y} z: {litwist_z} \n - Angular twist data -->x: {angtwist_x} y: {angtwist_y} z: {angtwist_z} \n \n")
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.position_data.append([current_time,pos_x,pos_y,pos_z])
        self.orientation_data.append([current_time,orient_x,orient_y,orient_z,orient_w])
        self.lineartwist_data.append([current_time,litwist_x,litwist_y,litwist_z])
        self.angulartwist_data.append([current_time,angtwist_x,angtwist_y,angtwist_z])
        self.message_buffer.clear()
        #in the above 4 line of code, based on the type of the data (whether it is position,orientation, linear twist, angular twist data), it is appended in the corresponding lists. 




    def save_to_excel(self):
        # Append collected data to the sheet
        for a in self.position_data:
            self.position_worksheet.append(a)

        for b in self.orientation_data:
            self.orientation_worksheet.append(b)

        for c in self.lineartwist_data:
            self.lineartwist_worksheet.append(c)

        for d in self.angulartwist_data:
            self.angulartwist_worksheet.append(d)

        # Save the workbook
        self.workbook.save('odom_data.xlsx')
        self.get_logger().info('Data saved to odom_data.xlsx')

    def destroy_node(self):
        
        # Save data to Excel before shutting down the node
        self.get_logger().info('Saving data to Excel...')
        self.save_to_excel()
        super().destroy_node()
        



def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    
    try:    
         rclpy.spin(odom_subscriber)
        # the odom_subscriber node will start receiving and processing the messages indefinitely in a loop, until it is stopped manualy 
    except KeyboardInterrupt:  #when the user press ctrl+c to stop the node
         
         pass
    finally: #finally makes sure that task written under it will be followed after try and exception blocks, regardless of whether an eexception was raised or not
        if odom_subscriber:  # this if checks whether odom_subscriber exist or not
            odom_subscriber.destroy_node()    #function to destroy the node is called 
        if rclpy.ok():    #this checks if rclpy is still running. If it is running then rclpy.shutdown will be executed
            rclpy.shutdown()
     


if __name__ == '__main__':
    main()
