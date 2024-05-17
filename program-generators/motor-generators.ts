import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";

const motorOptionToMotorName = new Map()
    .set("THUMB_LEFT_OPPOSITION", "thumb_left_opposition")
    .set("THUMB_LEFT_STRETCH", "thumb_left_stretch")
    .set("INDEX_LEFT_STRETCH", "index_left_stretch")
    .set("MIDDLE_LEFT_STRETCH", "middle_left_stretch")
    .set("RING_LEFT_STRETCH", "ring_left_stretch")
    .set("PINKY_LEFT_STRETCH", "pinky_left_stretch")
    .set("ALL_FINGERS_LEFT_STRETCH", "all_fingers_left")
    .set("THUMB_RIGHT_OPPOSITION", "thumb_right_opposition")
    .set("THUMB_RIGHT_STRETCH", "thumb_right_stretch")
    .set("INDEX_RIGHT_STRETCH", "index_right_stretch")
    .set("MIDDLE_RIGHT_STRETCH", "middle_right_stretch")
    .set("RING_RIGHT_STRETCH", "ring_right_stretch")
    .set("PINKY_RIGHT_STRETCH", "pinky_right_stretch")
    .set("ALL_FINGERS_RIGHT_STRETCH", "all_fingers_right")
    .set("UPPER_ARM_LEFT_ROTATION", "upper_arm_left_rotation")
    .set("ELBOW_LEFT", "elbow_left")
    .set("LOWER_ARM_LEFT_ROTATION", "lower_arm_left_rotation")
    .set("WRIST_LEFT", "wrist_left")
    .set("SHOULDER_VERTICAL_LEFT", "shoulder_vertical_left")
    .set("SHOULDER_HORIZONTAL_LEFT", "shoulder_horizontal_left")
    .set("UPPER_ARM_RIGHT_ROTATION", "upper_arm_right_rotation")
    .set("ELBOW_RIGHT", "elbow_right")
    .set("LOWER_ARM_RIGHT_ROTATION", "lower_arm_right_rotation")
    .set("WRIST_RIGHT", "wrist_right")
    .set("SHOULDER_VERTICAL_RIGHT", "shoulder_vertical_right")
    .set("SHOULDER_HORIZONTAL_RIGHT", "shoulder_horizontal_right")
    .set("TILT_FORWARD_HEAD", "tilt_forward_motor")
    .set("TURN_HEAD", "turn_head_motor");

const generateRosNodeClassDefinition = (className: string) => `
class ${className}(Node):
    def __init__(self):                
        super().__init__('joint_trajectory_publisher')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', qos_profile=qos_policy)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.frame_id = 'default_frame'
        msg.header.stamp.sec = round(time.time())
        msg.joint_names = [selected_motor]
        point = JointTrajectoryPoint()
        point.positions = [motor_name_to_position.get(selected_motor, 0)]
        point.velocities = [16000.0]
        point.accelerations = [10000.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 10000000
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

`;

// add reserved keywords, to prevent variavle names in python-code to be overwritten
pythonGenerator.addReservedWords(
    "motor_name_to_position,selected_motor,joint_trajectory_publisher,JointTrajectoryPublisher()",
);

export function move_motor(block: Block, generator: typeof pythonGenerator) {
    // extract block-input
    const motorOption = <string>block.getFieldValue("MOTORNAME");
    const modeInput = block.getFieldValue("MODE");
    const positionInput = String(
        generator.valueToCode(block, "POSITION", Order.ATOMIC),
    );
    const selectedMotorName: string = motorOptionToMotorName.get(motorOption);
    if (selectedMotorName === undefined) {
        throw new Error(
            `'${selectedMotorName}' is not a valid value for 'MOTORNAME'.`,
        );
    }

    // declare python imports
    (generator as any).definitions_["import_rclpy"] = "import rclpy";
    (generator as any).definitions_["from_rclpy_node_import_Node"] =
        "from rclpy.node import Node";
    (generator as any).definitions_["import_time"] = "import time";
    (generator as any).definitions_[
        "from_trajectory_msgs_msg_import_JointTrajectory_JointTrajectoryPoint"
    ] = "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint";

    // generate code that creates an empty dictionary where motor positions are stored
    (generator as any).definitions_[
        "motor_name_to_position"
    ] = `motor_name_to_position = {}`;

    // declare the 'JointTrajectoryPublisher'-class
    generator.provideFunction_(
        "JointTrajectoryPublisher",
        generateRosNodeClassDefinition(generator.FUNCTION_NAME_PLACEHOLDER_),
    );

    // initialize rclpy and instantiate the 'JointTrajectoryPublisher'-node
    (generator as any).definitions_["rclpy_init"] = `rclpy.init()`;
    (generator as any).definitions_[
        "joint_trajectory_publisher_=_JointTrajectoryPublisher()"
    ] = `joint_trajectory_publisher = JointTrajectoryPublisher()`;
    (generator as any).definitions_["selected_motor"] = `selected_motor = None`;

    // generate code for computing the target postion of the selected motor
    let positionString = "";
    if (modeInput == "ABSOLUTE") {
        positionString = positionInput;
    } else if (modeInput == "RELATIVE") {
        positionString =
            "motor_name_to_position.get('" +
            selectedMotorName +
            "', 0) + " +
            positionInput;
    } else {
        throw new Error(`unexpected input-mode: ${modeInput}.`);
    }

    // generate code for publishing the target motor-positiom
    const code =
        "selected_motor = '" +
        selectedMotorName +
        "'\n" +
        "motor_name_to_position['" +
        selectedMotorName +
        "'] = float(" +
        positionString +
        ")\n" +
        "rclpy.spin_once(joint_trajectory_publisher)\n";

    return code;
}

export {pythonGenerator};
