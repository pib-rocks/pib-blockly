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
    const prompt = generator.valueToCode(block, "TEXT", Order.ATOMIC);
    const type = <string>block.getFieldValue("TYPE");

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
    let code = "";
    switch (type) {
        case 'TEXT':
            code = `${functionName}(${prompt})\n`;
            break;
        case 'NUMBER':
            code = `float(${functionName}(${prompt}))\n`;
            break;
        default:
            throw new Error(`'${type}' is not a valid input-type.`);
    }
    return [code, Order.ATOMIC];
}

export {pythonGenerator};
