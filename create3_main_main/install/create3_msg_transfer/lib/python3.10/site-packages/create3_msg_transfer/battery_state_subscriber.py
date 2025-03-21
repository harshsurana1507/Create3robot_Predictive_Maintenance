import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from openpyxl import Workbook
from datetime import datetime



class BatteryStateSubscriber(Node):
    def __init__(self):
        super().__init__('battery_state_subscriber')

        topic_name = '/robot01/battery_state'
        #change /robot01 with the name of the robot kept
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            BatteryState,
            topic_name,
            self.battery_state_callback,
            qos_profile
            )
        self.subscription  # prevent unused variable warning
        self.workbook = Workbook()
        self.worksheet = self.workbook.active
        self.worksheet.title = 'Battery Status'
        self.worksheet.append(["Timestamp", "Voltage (V)", "Current (A)","Charge (Ah)","Capacity (Ah)","Temperature","Percentage %"])
        # above append line is for adding a header file in the excel sheet, below which the sensor readings will be added

        self.data = []    #all the data which will be added in the excel sheet, are first added in this list, then once the node is terminated
        # all the data from the list will be then saved in the excel file.

        self.timer = self.create_timer(1.0, self.process_buffered_messages)    
        # in gap of 1 seconds the function process_buffered_messages will be called, so that the latest message from the topic which
        # was stored in the list self.message_buffer can be extracted and stored in the data list.
        self.message_buffer = []    # a list is created so that all the readings from the topic can be stored in it.
        



    def battery_state_callback(self, msg):
        self.message_buffer.append(msg)

    def process_buffered_messages(self):
        if not self.message_buffer: 
            return
        # Process the latest message from the buffer list
        latest_msg = self.message_buffer[-1]
        #self.get_logger().info(str(msg))
        self.get_logger().info(f"Battery State: \n - Voltage: {latest_msg.voltage} V\n - Current: {latest_msg.current} A\n - Charge: {latest_msg.charge} Ah\n - Capacity: {latest_msg.capacity} Ah \n - Temperature: {latest_msg.temperature}\n - Percentage: {latest_msg.percentage * 100}% \n \n")
        #the above command displays the readings of the sensor on the screen
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.data.append([current_time, latest_msg.voltage, latest_msg.current, latest_msg.charge, latest_msg.capacity,latest_msg.temperature,(latest_msg.percentage*100)])
        
        self.message_buffer.clear()
        # the buffer list which was created is cleared, so as to store new set of data.

    def save_to_excel(self):
        # Append collected data to the sheet
        for row in self.data:
            self.worksheet.append(row)

        # Save the workbook
        self.workbook.save("battery_status.xlsx")
        self.get_logger().info('Data saved to battery_status.xlsx')

    def destroy_node(self):
        
        # Save data to Excel before shutting down the node
        self.get_logger().info('Saving data to Excel...')
        self.save_to_excel()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    battery_state_subscriber = BatteryStateSubscriber()
    
    try:
        rclpy.spin(battery_state_subscriber)
        # the battery state subscriber node will start receiving and processing the messages indefinitely in a loop, until it is stopped manualy 

    except KeyboardInterrupt:   #when the user press ctrl+c to stop the node
         
         pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:  #finally makes sure that task written under it will be followed after try and exception blocks, regardless of whether an eexception was raised or not
        if battery_state_subscriber:      # this if checks whether battery_state_subscriber exist or not
            battery_state_subscriber.destroy_node()   #function to destroy the node is called 
        if rclpy.ok():  #this checks if rclpy is still running. If it is running then rclpy.shutdown will be executed
            rclpy.shutdown()




if __name__ == '__main__':
    main()