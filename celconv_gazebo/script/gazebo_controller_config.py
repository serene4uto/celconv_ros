from argparse import ArgumentParser
from ament_index_python.packages import get_package_share_directory
import yaml

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('prefix', type=str, default='')
    parser.add_argument('num_rows', type=int, default=1)
    parser.add_argument('num_cols', type=int, default=1)
    args = parser.parse_args()

    prefix = args.prefix
    num_rows = args.num_rows
    num_cols = args.num_cols

    joint_list = []
    for r in range(num_rows):
        for c in range(num_cols):
            joint_list.append(f'rim_left_to_cell_{r*num_cols + c}_base_joint')
            joint_list.append(f'rim_back_to_cell_{r*num_cols + c}_base_joint')
            joint_list.append(f'rim_right_to_cell_{r*num_cols + c}_base_joint')


    save_config_file = get_package_share_directory('celconv_control') + f'/config/{prefix}_gazebo_control.yaml'

    control_config = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': int(20),
                f'celconv_velocity_controller': {
                    'type': 'velocity_controllers/JointGroupVelocityController',
                },

                f'celconv_joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster',
                },
            }
        },
        f'celconv_velocity_controller': {
            'ros__parameters': {
                'joints': joint_list
            }
        },
    }

    with open(save_config_file, 'w') as f:
        yaml.dump(control_config, f)
    
    