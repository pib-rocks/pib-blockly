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

const APPLY_JOINT_TRAJECTORY_DEF = `
def apply_joint_trajectory(motor_name: str, position: int) -> None:

    logging.info(f"setting position of '{motor_name}' to {position}.")

    request = ApplyJointTrajectory.Request()
    point = JointTrajectoryPoint()
    point.positions.append(position)
    jt = JointTrajectory()
    jt.joint_names = [motor_name]
    jt.points = [point]
    request.joint_trajectory = jt

    future = apply_joint_trajectory_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response: ApplyJointTrajectory.Response = future.result()
    if response.successful:
        logging.info(f"position of '{motor_name}' was successfully set.")
        motor_name_to_position[motor_name] = position
    else:
        logging.error(f"setting position of '{motor_name}' failed.")
`;

const CONFIGURE_LOGGING = `
# configure the python-logger
stdout_handler = logging.StreamHandler(sys.stdout)
stdout_handler.setLevel(logging.INFO)
stdout_handler.addFilter(lambda rec: rec.levelno <= logging.INFO)
stderr_handler = logging.StreamHandler()
stderr_handler.setLevel(logging.WARNING)
logging.basicConfig(
    level=logging.INFO,
    handlers=[stdout_handler, stderr_handler],
    format="[%(levelname)s] [%(asctime)s]: %(message)s",
    datefmt="%y-%m-%d %H:%M:%S"
)
`;

const INIT_ROS = `
rclpy.init()
node = rclpy.create_node("blockly_node")
`;

const INIT_CLIENT = `
apply_joint_trajectory_client = node.create_client(
    ApplyJointTrajectory,
    'apply_joint_trajectory'
)

logging.info(f"waiting for 'apply_joint_trajectory' service to become available...")
apply_joint_trajectory_client.wait_for_service()
logging.info(f"service now available")
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
    generator.definitions_["import_rclpy"] = "import rclpy";
    generator.definitions_["import_sys"] = "import sys";
    generator.definitions_["import_logging"] = "import logging";
    generator.definitions_["import_ApplyJointTrajectory"] = "from datatypes.srv import ApplyJointTrajectory";
    generator.definitions_["from_trajectory_msgs_msg_import_JointTrajectory_JointTrajectoryPoint"] = "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint";

    // configure logging
    generator.definitions_["configure_logging"] = CONFIGURE_LOGGING;
    
    // initialize rclpy and create node
    generator.definitions_["init_ros"] = INIT_ROS;

    // initialize apply-joint-trajectory-client
    generator.definitions_["init_apply_joint_trajectory_client"] = INIT_CLIENT;

    // generate code that creates an empty dictionary where motor positions are stored
    generator.definitions_[
        "motor_name_to_position"
    ] = `motor_name_to_position = {}`;

    // declare the 'apply_joint_trajectory'-function
    generator.provideFunction_(
        "apply_joint_trajectory",
        APPLY_JOINT_TRAJECTORY_DEF,
    );

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

    return `apply_joint_trajectory("${selectedMotorName}", ${positionString})\n`;
}

export {pythonGenerator};
