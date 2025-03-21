import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from openpyxl import Workbook
from datetime import datetime

class CliffIntensitySubscriber(Node):
    def __init__(self):
        super().__init__('cliff_intensity_subscriber')

        topic_name = '/robot01/cliff_intensity'
        #change /robot01 with the name of the robot kept
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            IrIntensityVector,
            topic_name,
            self.cliff_intensity_callback,
            qos_profile
            )
        self.subscription  # prevent unused variable warning
        self.workbook = Workbook()
        self.worksheet = self.workbook.active
        self.worksheet.title = 'Cliff Intensity'
        self.frame_ids = ["cliff_side_left", "cliff_front_left", "cliff_front_right", "cliff_side_right"]

        self.worksheet.append(["Timestamp"] + self.frame_ids)
        # above append line is for adding a header file in the excel sheet, below which the sensor readings will be added

        self.data = []  #all the data which will be added in the excel sheet, are first added in this list, then once the node is terminated
        # all the data from the list will be then saved in the excel file.
        self.timer = self.create_timer(1.0, self.process_buffered_messages)
        # in gap of 1 seconds the function process_buffered_messages will be called, so that the latest message from the topic which
        # was stored in the list self.message_buffer can be extracted and stored in the data list.
        self.message_buffer = [] # a list is created so that all the readings from the topic can be stored in it.

    

    def cliff_intensity_callback(self, msg):
        self.message_buffer.append(msg)   

    def process_buffered_messages(self):
        if not self.message_buffer: #this checks if self.message_buffer has any data/messages or not
            return

        # Process the latest message from the buffer
        latest_msg = self.message_buffer[-1]

        # Current time and date
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Create a dictionary to hold the readings for this timestamp
        row_data = {"Timestamp": current_time}
        for frame_id in self.frame_ids:   #it has initialised the row_data dictionary with keys as each frame id found in self.frame_ids and initially the value is set to none
            row_data[frame_id] = None

        for intensity in latest_msg.readings:
            frame_id = intensity.header.frame_id
            ir_value = intensity.value    #intensity.value is the data read by different cliff sensors
            row_data[frame_id] = ir_value   #in the row data dictionary, the ir_value is stored with the frame_id key 
            self.get_logger().info(f"Cliff Intensity: \n - Frame ID: {frame_id} \n - the value is {ir_value} \n \n ----")

        # Append the row data to self.data
        self.data.append(row_data)
        self.message_buffer.clear()

    def save_to_excel(self):
        # Append collected data to the sheet
        for row in self.data:
            self.worksheet.append([row["Timestamp"]] + [row[frame_id] for frame_id in self.frame_ids])
            #adding the timestamp and the sensor readings in the worksheet


        # Save the workbook
        self.workbook.save('cliff_intensity.xlsx')
        self.get_logger().info('Data saved to cliff_intensity.xlsx')

    def destroy_node(self):
        # Save data to Excel before shutting down the node
        self.get_logger().info('Saving data to Excel...')
        self.save_to_excel()
        super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    cliff_intensity_subscriber = CliffIntensitySubscriber()
    
    try:
        rclpy.spin(cliff_intensity_subscriber)
        # the cliff state subscriber node will start receiving and processing the messages indefinitely in a loop, until it is stopped manualy 
    except KeyboardInterrupt:  #when the user press ctrl+c to stop the node
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    
    
    
    finally: #finally makes sure that task written under it will be followed after try and exception blocks, regardless of whether an eexception was raised or not
        if cliff_intensity_subscriber:  # this if checks whether cliff_intensity_subscriber exist or not
            cliff_intensity_subscriber.destroy_node()   #function to destroy the node is called 
        if rclpy.ok():       #this checks if rclpy is still running. If it is running then rclpy.shutdown will be executed
            rclpy.shutdown()

if __name__ == '__main__':
    main()
