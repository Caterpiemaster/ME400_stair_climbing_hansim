import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import inputs

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.num_axes = 6  # 4 sticks + 2 triggers
        self.num_buttons = 11  # A, B, X, Y, LB, RB, Back, Start, LS, RS
        self.current_axes = [0.0] * self.num_axes
        self.current_buttons = [0] * self.num_buttons

    def timer_callback(self):
        events = inputs.get_gamepad()
        updated = False

        for event in events:
            self.get_logger().info(f'Event: {event.ev_type}, {event.code}, {event.state}')
            updated = True

            if event.ev_type == 'Absolute':
                index = self._get_axis_index(event.code)
                if index is not None:
                    self.current_axes[index] = self._normalize_axis(event.code, event.state)

            elif event.ev_type == 'Key':
                index = self._get_button_index(event.code)
                if index is not None:
                    self.current_buttons[index] = event.state

        if updated or not events:
            joy_msg = Joy()
            joy_msg.axes = self.current_axes
            joy_msg.buttons = self.current_buttons
            self.publisher_.publish(joy_msg)
            self.get_logger().info(f'Published Joy message: axes={joy_msg.axes}, buttons={joy_msg.buttons}')

    def _normalize_axis(self, code, value):
        if code in ['ABS_Z', 'ABS_RZ']:
            # Triggers can be from 0 to 255 on some gamepads
            return value / 255.0
        else:
            # Normalize to range -1.0 to 1.0
            return value / 32767.0 if value >= 0 else value / 32768.0

    def _get_axis_index(self, code):
        axis_mapping = {
            'ABS_X': 0,      # Left stick X
            'ABS_Y': 1,      # Left stick Y
            'ABS_RX': 2,     # Right stick X
            'ABS_RY': 3,     # Right stick Y
            'ABS_Z': 4,      # Left trigger
            'ABS_RZ': 5,     # Right trigger
        }
        return axis_mapping.get(code, None)

    def _get_button_index(self, code):
        button_mapping = {
            'BTN_SOUTH': 0,  # A
            'BTN_EAST': 1,   # B
            'BTN_WEST': 2,   # X
            'BTN_NORTH': 3,  # Y
            'BTN_TL': 4,     # LB
            'BTN_TR': 5,     # RB  
            'BTN_SELECT': 6, # Back
            'BTN_START': 7,  # Start
            'BTN_THUMBL': 8, # Left stick click
            'BTN_THUMBR': 9, # Right stick click
            'BTN_MODE': 10,  # Mode button (if present)
        }
        return button_mapping.get(code, None)

def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()

def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()

    rclpy.spin(joystick_node)

    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
