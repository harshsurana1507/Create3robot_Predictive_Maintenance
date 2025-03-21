import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from openpyxl import Workbook
from datetime import datetime

class DockStatusSubscriber(Node):
    def __init__(self):
        super().__init__('dock_status_subscriber')

        topic_name = '/robot01/dock_status'
        #change /robot01 with the name of the robot kept
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            DockStatus,
            topic_name,
            self.dock_status_callback,
            qos_profile
            )
        self.subscription  # prevent unused variable warning
        self.workbook = Workbook()
        self.worksheet = self.workbook.active
        self.worksheet.title = 'Dock Status'
        self.worksheet.append(["Timestamp", "Is it Docked", "Is Dock Visible"])
        # above append line is for adding a header file in the excel sheet, below which the sensor readings will be added
        
        self.data = [] #all the data which will be added in the excel sheet, are first added in this list, then once the node is terminated
        # all the data from the list will be then saved in the excel file.
        
        self.timer = self.create_timer(1.0, self.process_buffered_messages)
        # in gap of 1 seconds the function process_buffered_messages will be called, so that the latest message from the topic which
        # was stored in the list self.message_buffer can be extracted and stored in the data list.
        
        self.message_buffer = [] # a list is created so that all the readings from the topic can be stored in it.



    def dock_status_callback(self, msg):
        self.message_buffer.append(msg)

    
    def process_buffered_messages(self):
        if not self.message_buffer:
            return

        # Process the latest message from the buffer
        latest_msg = self.message_buffer[-1]
        #self.get_logger().info(str(msg))
        self.get_logger().info(f"Dock status: \n - is robot connected to charger: {latest_msg.is_docked} \n - is charging dock visible: {latest_msg.dock_visible} \n \n")
        #current time and date
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        if latest_msg.is_docked == True:
            a = "1"
        
        if latest_msg.is_docked == False:
            a="0"
        
        if latest_msg.dock_visible == True:
            b="1"
        
        if latest_msg.dock_visible == False:
            b="0"
            

        
        
        self.data.append([current_time, a, b])
        self.message_buffer.clear()
        # the buffer list which was created is cleared, so as to store new set of data.

        

    def save_to_excel(self):
        # Append collected data to the sheet
        for row in self.data:
            self.worksheet.append(row)

        # Save the workbook
        self.workbook.save("dock_status.xlsx")
        self.get_logger().info('Data saved to dock_status.xlsx')

    def destroy_node(self):
        
        # Save data to Excel before shutting down the node
        self.get_logger().info('Saving data to Excel...')
        self.save_to_excel()
        super().destroy_node()
    



def main(args=None):
    rclpy.init(args=args)
    dock_status_subscriber = DockStatusSubscriber()
    #the main class DockStatusSubscriber is called
    try:
        rclpy.spin(dock_status_subscriber)
        # the dock_status_subscriber node will start receiving and processing the messages indefinitely in a loop, until it is stopped manualy 
    except KeyboardInterrupt: #when the user press ctrl+c to stop the node
         
         pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:  #finally makes sure that task written under it will be followed after try and exception blocks, regardless of whether an eexception was raised or not
        if dock_status_subscriber: # this if checks whether dock_status_subscriber exist or not
            dock_status_subscriber.destroy_node()   #function to destroy the node is called
        if rclpy.ok():  #this checks if rclpy is still running. If it is running then rclpy.shutdown will be executed
            rclpy.shutdown()



if __name__ == '__main__':
    main()
