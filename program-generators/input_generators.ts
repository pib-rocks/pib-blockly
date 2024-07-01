import {Block} from "blockly/core/block";

import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    CREATE_INPUT_QUEUE,
    CREATE_PROMPT_PUBLISHER_AND_SUBSCRIBER,
    GET_MPID,
    IMPORT_LOGGING,
    IMPORT_PROGRAM_INPUT,
    IMPORT_PROGRAM_PROMPT,
    IMPORT_PUBLISHER,
    IMPORT_QUEUE,
    IMPORT_RCLPY,
    IMPORT_SYS,
    INIT_ROS,
} from "./util/definitions";
import {RECEIVE_INPUT_FROM_USER} from "./util/function-declarations";

export function inputGenerator(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const prompt = <string>block.getFieldValue("PROMPT");
    const type = generator.valueToCode(block, "TYPE", Order.ATOMIC);

    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_PUBLISHER,
        IMPORT_SYS,
        IMPORT_LOGGING,
        IMPORT_QUEUE,
        IMPORT_PROGRAM_INPUT,
        IMPORT_PROGRAM_PROMPT,
        CONFIGURE_LOGGING,
        INIT_ROS,
        CREATE_INPUT_QUEUE,
        GET_MPID,
        CREATE_PROMPT_PUBLISHER_AND_SUBSCRIBER,
    });

    // declare the 'play_audio_from_speech'-function
    const functionName = generator.provideFunction_(
        "receive_input_from_user",
        RECEIVE_INPUT_FROM_USER(generator),
    );

    // generate code
    switch (type) {
        case 'str':
            return `${functionName}(${prompt})\n`;
        case 'int':
            return `int(${functionName}(${prompt}))\n`
        default:
            throw new Error(`'${type}' is not a valid input-type.`);
    }
}

export {pythonGenerator};
