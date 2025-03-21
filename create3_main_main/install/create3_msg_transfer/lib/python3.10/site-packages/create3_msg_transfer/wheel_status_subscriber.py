import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import WheelStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from openpyxl import Workbook
from datetime import datetime

class WheelStatusSubscriber(Node):
    def __init__(self):
        super().__init__('wheel_status_subscriber')

        topic_name = '/robot01/wheel_status'
        #change /robot01 with the name of the robot kept

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            WheelStatus,
            topic_name,
            self.wheel_status_callback,
            qos_profile
            )
        self.subscription  # prevent unused variable warning
        self.workbook = Workbook()
        self.worksheet = self.workbook.active
        self.worksheet.title = 'wheel status'
        self.worksheet.append(["Timestamp","current_ma_left","current_ma_right","pwm_left","pwm_right"])

        self.data = []
        #all the data which will be added in the excel sheet, are first added in this list, then once the node is terminated
        # all the data from the list will be then saved in the excel file.

        self.timer = self.create_timer(1.0, self.process_buffered_messages)
        # in gap of 1 seconds the function process_buffered_messages will be called, so that the latest message from the topic which
        # was stored in the list self.message_buffer can be extracted and stored in the data list.
        
        
        self.message_buffer = []
        # a list is created so that all the readings from the topic can be stored in it.


    def wheel_status_callback(self, msg):
        self.message_buffer.append(msg)

    def process_buffered_messages(self):
        if not self.message_buffer:
            return
        latest_msg = self.message_buffer[-1]

        #self.get_logger().info(str(msg))
        self.get_logger().info(f"Wheel status data: \n -current_ma_left: {latest_msg.current_ma_left} \n -current_ma_right: {latest_msg.current_ma_right} \n -pwm_left: {latest_msg.pwm_left} \n -pwm_right: {latest_msg.pwm_right} \n\n")
        #current time and date
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        self.data.append([current_time,latest_msg.current_ma_left,latest_msg.current_ma_right,latest_msg.pwm_left,latest_msg.pwm_right])
        self.message_buffer.clear()

    def save_to_excel(self):
        # Append collected data to the sheet
        for row in self.data:
            self.worksheet.append(row)

        # Save the workbook
        self.workbook.save('wheel status.xlsx')
        self.get_logger().info('Data saved to wheel status.xlsx')

    def destroy_node(self):
        
        # Save data to Excel before shutting down the node
        self.get_logger().info('Saving data to Excel...')
        self.save_to_excel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    wheel_status_subscriber = WheelStatusSubscriber()
    try:
        rclpy.spin(wheel_status_subscriber)

    except KeyboardInterrupt:   #when the user press ctrl+c to stop the node
         
         pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:  #finally makes sure that task written under it will be followed after try and exception blocks, regardless of whether an exception was raised or not
        if wheel_status_subscriber:  # this if checks whether wheel_status_subscriber exist or not
            wheel_status_subscriber.destroy_node()  #function to destroy the node is called 
        if rclpy.ok():   #this checks if rclpy is still running. If it is running then rclpy.shutdown will be executed
            rclpy.shutdown()

if __name__ == '__main__':
    main()
