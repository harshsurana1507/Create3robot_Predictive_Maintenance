import pandas as pd
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from openpyxl import Workbook
from datetime import datetime




class TfSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')

        topic_name = '/robot01/tf'
        #change /robot01 with the name of the robot kept
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            TFMessage,
            topic_name,
            self.tf_callback,
            qos_profile
            )
        self.subscription  # prevent unused variable warning
        self.workbook = Workbook()
        self.baselink_worksheet = self.workbook.active
        self.baselink_worksheet.title = 'base link data '
        self.baselink_worksheet.append(["Timestamp", "link_trans_X", "link_trans_Y","link_trans_Z","link_rot_X","link_rot_Y","link_rot_Z","link_rot_W"])

        self.basefootprint_worksheet = self.workbook.create_sheet(title= 'base footprint data ')
        self.basefootprint_worksheet.append(["Timestamp", "footprint_trans_X", "footprint_trans_Y","footprint_trans_Z","footprint_rot_X","footprint_rot_Y","footprint_rot_Z","footprint_rot_W"])
        # 2 seperate sheets are created for clarity of base link data and base footprint data



        self.baselink_data = []
        self.basefootprint_data = []
        #all the data which will be added in the excel sheets, are first added in these list, then once the node is terminated
        # all the data from the list will be then saved in the excel file.



        self.timer = self.create_timer(1.0, self.process_buffered_messages)
        # in gap of 1 seconds the function process_buffered_messages will be called, so that the latest message from the topic which
        # was stored in the lists self.message_buffer can be extracted and stored in the data list.
        
        
        self.message_buffer = []
        # a list is created so that all the readings from the topic can be stored in it.


    def tf_callback(self, msg):
        self.message_buffer.append(msg)

        
    def process_buffered_messages(self):
        if not self.message_buffer:
            return
        
        latest_msg = self.message_buffer[-1]
        
        #self.get_logger().info(str(msg))
        

        for transform in latest_msg.transforms:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if transform.child_frame_id == "base_link":
                trans_x = transform.transform.translation.x
                trans_y = transform.transform.translation.y
                trans_z = transform.transform.translation.z

                rot_x = transform.transform.rotation.x
                rot_y = transform.transform.rotation.y  
                rot_z = transform.transform.rotation.z
                rot_w = transform.transform.rotation.w

                self.get_logger().info(f"TF data from base link: \n - Translation data --> x: {trans_x} y: {trans_y} z: {trans_z}\n - Rotation data-->  x:{rot_x} y: {rot_y} z: {rot_z} w:{rot_w}   \n")
                
                self.baselink_data.append([current_time,trans_x,trans_y,trans_z,rot_x,rot_y,rot_z,rot_w])



            if transform.child_frame_id == "base_footprint":
                trans_x_foot = transform.transform.translation.x
                trans_y_foot = transform.transform.translation.y
                trans_z_foot = transform.transform.translation.z

                rot_x_foot = transform.transform.rotation.x
                rot_y_foot = transform.transform.rotation.y  
                rot_z_foot = transform.transform.rotation.z
                rot_w_foot = transform.transform.rotation.w

                self.get_logger().info(f"TF data from base footprint: \n - Translation data --> x: {trans_x_foot} y: {trans_y_foot} z: {trans_z_foot}\n - Rotation data-->  x:{rot_x_foot} y: {rot_y_foot} z: {rot_z_foot} w:{rot_w_foot} --- ")
                
                self.basefootprint_data.append([current_time,trans_x_foot,trans_y_foot,trans_z_foot,rot_x_foot,rot_y_foot,rot_z_foot,rot_w_foot])
        self.message_buffer.clear()

    def save_to_excel(self):
        # Append collected data to the sheet
        for a in self.baselink_data:
            self.baselink_worksheet.append(a)

        for b in self.basefootprint_data:
            self.basefootprint_worksheet.append(b)

        # Save the workbook
        self.workbook.save('tf_data.xlsx')
        self.get_logger().info('Data saved to tf_data.xlsx')

    def destroy_node(self):
        
        # Save data to Excel before shutting down the node
        self.get_logger().info('Saving data to Excel...')
        self.save_to_excel()
        super().destroy_node()
        



def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TfSubscriber()
    
    try:    
         rclpy.spin(tf_subscriber) 

    except KeyboardInterrupt: #when the user press ctrl+c to stop the node
         
         pass
    finally:  #finally makes sure that task written under it will be followed after try and exception blocks, regardless of whether an exception was raised or not
        if tf_subscriber:  # this if checks whether tf_subscriber exist or not
            tf_subscriber.destroy_node()   #function to destroy the node is called 
        if rclpy.ok():   #this checks if rclpy is still running. If it is running then rclpy.shutdown will be executed
            rclpy.shutdown()
     


if __name__ == '__main__':
    main()
