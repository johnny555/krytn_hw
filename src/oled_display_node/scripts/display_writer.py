#!/usr/bin/env python3

"""
Copyright (c) 2019, Ubiquity Robotics
All rights reserved.
# ... (License text remains the same) ...
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Example client program for writing to display using ROS2 topic
"""

import rclpy
from rclpy.node import Node

# Import the custom message from the ROS2 package
from oled_display_node.msg import DisplayOutput

import time

class DisplayWriterNode(Node):
    """
    Constructor for our class
    """
    def __init__(self):
        super().__init__('display_writer')

        # A publisher for sending commands to the display node
        # Topic name matches the subscriber in the C++ node
        self.display_cmd_pub = self.create_publisher(DisplayOutput, "display_output", 10)
        self.get_logger().info('Display Writer Node started, ready to send commands.')

    def publish_display_command(self, row, column, text, comment):
        # Ensure text fits (assuming max 16 columns like in C++ node context)
        max_chars = 16 # Should ideally match display capabilities
        num_chars = len(text)
        if num_chars > max_chars:
            text = text[:max_chars]
            num_chars = max_chars

        disp_msg = DisplayOutput()
        # Use constants defined in the message definition
        disp_msg.action_type = DisplayOutput.DISPLAY_SUBSTRING # Renamed actionType
        disp_msg.row = row             # Display row with 0 as top row
        disp_msg.column = column       # Display column (character column, C++ node converts to pixels)
        disp_msg.num_chars = num_chars # Renamed numChars
        disp_msg.attributes = 0        # Just write with no attributes
        disp_msg.text = text           # The text to be written
        disp_msg.comment = comment     # Comment in the message, no functional use

        self.display_cmd_pub.publish(disp_msg)
        self.get_logger().info(f'Published to row {row}, col {column}: "{text}"')

    """
    Function to send the sequence of messages
    """
    def send_sequence(self):
        self.get_logger().info("ROS2 publisher sending commands to display topic")

        # Allow time for publisher to establish connection
        time.sleep(1.0)

        self.publish_display_command(3, 2, "Data for the", "Write to a line")
        time.sleep(1.0)
        self.publish_display_command(4, 2, "Application ", "Write to a line")
        time.sleep(1.0)
        self.publish_display_command(5, 2, "will now be ", "Write to a line")
        time.sleep(1.0)
        self.publish_display_command(6, 2, "written to  ", "Write to a line")
        time.sleep(1.0)
        self.publish_display_command(7, 2, "OLED display", "Write to a line")
        time.sleep(1.0) # Allow last message to be sent

        self.get_logger().info("Commands sent.")


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of our node class
    node = DisplayWriterNode()

    try:
        # Run the message sending sequence
        node.send_sequence()
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        # Shutdown
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("Display Writer Node shut down.")


if __name__ == "__main__":
    main()
