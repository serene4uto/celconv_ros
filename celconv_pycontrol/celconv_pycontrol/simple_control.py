import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

INIT_STATE = 0
IDLE_STATE = 1
RUNNING_STATE = 2


# moveBindings = {
#     'i':(1,0,0),
#     'o':(1,-1,0),
#     'j':(1,1,1),
#     'l':(-1,-1,-1),
#     'u':(-1,0,1),
#     ',':(-1,0,0),
#     '.':(1,0,-1),
#     'm':(-1,1,0),  
# }

MOVE_BINDINGS = [
    (1, 0, 0), (1, -1, 0), (1, 1, 1), (-1, -1, -1), (-1, 0, 1), (-1, 0, 0), (1, 0, -1), (-1, 1, 0)
]
class CelconvSimpleControl(Node):
    def __init__(self):
        super().__init__('celconv_simple_control')
        self.get_logger().info('CelconvSimpleControl Node has been started.')

        self.states = INIT_STATE
        self.num_cells = 0

        self.state_manager_timer = self.create_timer(0.1, self.on_state_manager_timer_callback)

    def on_state_manager_timer_callback(self):
        if self.states == INIT_STATE:
            self.joint_sub = self.create_subscription(JointState, 'joint_states', self.on_joint_states_callback, 10)
            self.vel_pub = self.publishers(Float64MultiArray, '/celconv_velocity_controller/commands', 10)
            self.cell_control_sub = self.create_subscription(Float64MultiArray, 'cell_control', self.on_cell_control_callback, 10)
            self.states = IDLE_STATE

        elif self.states == IDLE_STATE:
            self.states = RUNNING_STATE

        elif self.states == RUNNING_STATE:
            pass
    
    def on_cell_control_callback(self, msg):
        if self.states == RUNNING_STATE:
            for i in range(len(msg.data)):
                if msg.data[i] == 1:
                    self.get_logger().info(f'Cell {i} is activated.')
                    self.publish_velocity(MOVE_BINDINGS[i])



    def on_joint_states_callback(self, msg):
        joint_name = msg.name
        for name in joint_name:
            if 'rim_back' in name and 'roller' not in name:
                self.num_cells += 1

        self.get_logger().info(f'Number of cells: {self.num_cells}')

        self.destroy_subscription(self.joint_sub)
        self.joint_sub = None
        
        
        





def main(args=None):
    rclpy.init(args=args)
    celconv_simple_control = CelconvSimpleControl()
    rclpy.spin(celconv_simple_control)
    celconv_simple_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()