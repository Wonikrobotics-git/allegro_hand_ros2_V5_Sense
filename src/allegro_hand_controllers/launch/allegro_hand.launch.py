from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.conditions import IfCondition
import os
import getpass

def generate_launch_description():
    allegro_hand_controllers_share = get_package_share_directory('allegro_hand_controllers')
    allegro_node_grasp_path = os.path.join(get_package_prefix('allegro_hand_controllers'), 'lib', 'allegro_hand_controllers', 'allegro_node_grasp')

    # Declare launch argument
    declare_visualize_arg = DeclareLaunchArgument(
        'VISUALIZE',
        default_value='false',
        description='Flag to enable/disable visualization'
    )

	
    declare_gui_arg = DeclareLaunchArgument(
        'GUI',
        default_value='false',
        description='Flag to enable/disable GUI'
    )


    declare_hand_arg = DeclareLaunchArgument(
        'HAND',
        default_value='right',
        description='Specify which hand to use: right or left'
    )

    declare_polling_arg = DeclareLaunchArgument(
        'POLLING',
        default_value='true',
        description='true, false for polling the CAN communication'
    )
    
    declare_can_device_arg = DeclareLaunchArgument(
    	'CAN_DEVICE', 
    	default_value='can0',
    	description='Specify CAN port for control multi devices'
    )
    
    declare_num_arg = DeclareLaunchArgument(
    	'NUM',
    	default_value='0',
    	description='Specify AH num for control multi devices'
    )

    declare_can_arg = DeclareLaunchArgument(
        'CAN',
        default_value='true',
        description='Communication selector: true -> CAN, false -> UDP (Ethernet)'
    )

    declare_udp_addr_arg = DeclareLaunchArgument(
        'UDP_ADDR',
        default_value='192.168.0.50:7000:8000',
        description='IP:PEER_PORT:LOCAL_PORT for UDP mode (CAN=false). PEER_PORT=firmware port (7000), LOCAL_PORT=PC bind port (default 8000)'
    )

    declare_hinf_arg = DeclareLaunchArgument(
        'HINF_MODE',
        default_value='false',
        description='Enable H-inf onboard position control mode (true/false). '
                    'Firmware must already be in H-inf mode.'
    )

    declare_demo_arg = DeclareLaunchArgument(
        'DEMO',
        default_value='false',
        description='Run sensor visualizer only (no rviz2); overrides VISUALIZE for the GUI node'
    )

    def setup_can(context):
        use_can = context.launch_configurations['CAN'].lower() in ['true', '1', 'yes', 'on']
        if not use_can:
            print('CAN=false, skipping CAN setup')
            return []

        ros_distro = os.environ.get('ROS_DISTRO', 'humble')
        ros_lib_path = f"/opt/ros/{ros_distro}/lib"
        ros_ld_conf = f"/etc/ld.so.conf.d/ros-{ros_distro}.conf"

        username = getpass.getuser()
        can_port = context.launch_configurations['CAN_DEVICE']
        commands = [
            f"sudo ip link set {can_port} down",
            f"sudo ip link set {can_port} type can bitrate 1000000",
            f"sudo ip link set {can_port} up",
            f"sudo sh -c 'echo \"{username} - rtprio 99\" > /etc/security/limits.d/99-allegro-rt.conf'",
            f"sudo sh -c 'echo {ros_lib_path} > {ros_ld_conf}'",
            f"sudo ldconfig",
            f"sudo setcap cap_sys_nice=ep {allegro_node_grasp_path}"
        ]

        while True:
            password = getpass.getpass('Enter sudo password: ')
            success = True

            for cmd in commands:
                result = os.system(f'echo "{password}" | sudo -S {cmd}')
                if result != 0:
                    print(f"Command failed: {cmd}")
                    success = False
                    break

            if success:
                print(f'{can_port} setup completed')
                break
            else:
                print(f'{can_port} setup failed. Please try again.')

        return []
        
    urdf_path = PythonExpression([
        '"', allegro_hand_controllers_share, '/urdf/allegro_hand_description_', LaunchConfiguration('HAND'),'.urdf"'
    ])

    return LaunchDescription([
        declare_visualize_arg,
        declare_hand_arg,
        declare_polling_arg,
        declare_can_device_arg,
        declare_num_arg,
		declare_gui_arg,
        declare_can_arg,
        declare_udp_addr_arg,
        declare_hinf_arg,
        declare_demo_arg,
        OpaqueFunction(function=setup_can),
        Node(
            package='allegro_hand_controllers',
            executable='allegro_node_grasp',
            name=PythonExpression(["'allegro_node_grasp_", LaunchConfiguration('NUM'), "'"]),
            output='screen',
			emulate_tty=True,
            parameters=[{'hand_info/which_hand': LaunchConfiguration('HAND')},
            		    {'comm/CAN_CH': LaunchConfiguration('CAN_DEVICE')},
                        {'comm/COMM_TYPE': PythonExpression([
                            "'can' if '", LaunchConfiguration('CAN'),
                            "'.lower() in ['true','1','yes','on'] else 'udp'"
                        ])},
                        {'comm/UDP_ADDR': LaunchConfiguration('UDP_ADDR')},
                        {'hinf_mode': PythonExpression([
                            "True if '", LaunchConfiguration('HINF_MODE'),
                            "'.lower() in ['true','1','yes','on'] else False"
                        ])}],
            arguments=[LaunchConfiguration('POLLING')],
			remappings=[
				('allegroHand/lib_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/lib_cmd'"])),
            	('allegroHand/joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
                ('allegroHand/joint_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_cmd'"])),
                ('allegroHand/gain_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/gain_cmd'"])),
                ('allegroHand/envelop_torque',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/envelop_torque'"])),
				('allegroHand/tactile_sensors',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/tactile_sensors'"])),
                ('allegroHand/gff_rpy',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/gff_rpy'"])),
                ('allegroHand/envelop_torque',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/envelop_torque'"])),
                ('fingertip_arrow_markers',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/fingertip_arrow_markers'"])),
                ('forcechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/force_chg'"])),
                ('timechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/time_chg'"])),

			]
        ),
        Node(
            package='robot_state_publisher',
            name=PythonExpression(["'robot_state_publisher_", LaunchConfiguration('NUM'), "'"]),
            output='screen',
			emulate_tty=True,
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}],
            remappings=[
                ('tf', PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/tf'"])),
                ('joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
                ('robot_description', 'allegro_hand_description')
            ]
        ),
        # Include the allegro_viz.launch.py file if VISUALIZE is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_controllers_share, 'launch', 'allegro_viz.launch.py')),
            condition=IfCondition(LaunchConfiguration('VISUALIZE')),
            launch_arguments={'NUM': LaunchConfiguration('NUM')}.items()
        ),
        # Sensor visualization GUI (allegro_hand_sensor_visualizer) if DEMO is true
        Node(
            package='allegro_hand_sensor_visualizer',
            executable='allegro_hand_sensor_visualizer_node',
            name=PythonExpression(["'sensor_gui_", LaunchConfiguration('NUM'), "'"]),
            output='screen',
            condition=IfCondition(LaunchConfiguration('DEMO')),
            parameters=[{'hand_info/which_hand': LaunchConfiguration('HAND')}],
            remappings=[
                ('allegroHand/tactile_sensors', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/tactile_sensors'"])),
                ('/tf', PythonExpression(["'/allegroHand_", LaunchConfiguration('NUM'), "/tf'"])),
                ('fingertip_arrow_markers', PythonExpression(["'allegroHand_", LaunchConfiguration('NUM'), "/fingertip_arrow_markers'"]))
            ]
        ),
		Node(
            package='allegro_hand_gui',
            executable='allegro_hand_gui_node',
            name=PythonExpression(["'allegro_hand_gui_node_", LaunchConfiguration('NUM'), "'"]),
            output='screen',
            condition=IfCondition(LaunchConfiguration('GUI')),
   			remappings=[
                ('allegroHand/joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
				('allegroHand/lib_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/lib_cmd'"])),
                ('forcechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/force_chg'"])),
                ('timechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/time_chg'"])),
                ('allegroHand/envelop_torque',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/envelop_torque'"])),
                ('allegroHand/joint_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_cmd'"])),
                ('allegroHand/gain_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/gain_cmd'"]))
			]         
        )
    ])
