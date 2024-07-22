import {Block} from "blockly/core/block";
import { pythonGenerator } from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_RCLPY,
    IMPORT_SYS,
    IMPORT_APPLY_POSE,
    INIT_APPLY_POSE_CLIENT,
    INIT_ROS,
} from "./util/definitions";
import {APPLY_POSE_FUNCTION} from "./util/function-declarations";

export function moveToPoseGenerator(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const poseId = <string>block.getFieldValue("POSENAME");
    const poseName = "Testpose";
    // const poseName = <string>pythonGenerator.nameDB_.getName(block.getFieldValue("POSENAME"),NameType.VARIABLE);
    // Blockly.JavaScript.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE) ??
    // const poseName = ??
    
    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_LOGGING,
        IMPORT_APPLY_POSE,
        CONFIGURE_LOGGING,
        INIT_ROS,
        INIT_APPLY_POSE_CLIENT,
    });

    // declare the 'apply_pose'-function
    const functionName = generator.provideFunction_(
        "apply_pose",
        APPLY_POSE_FUNCTION(generator),
    );

    return `${functionName}("${poseId}", "${poseName}")\n`;
}

export {pythonGenerator};