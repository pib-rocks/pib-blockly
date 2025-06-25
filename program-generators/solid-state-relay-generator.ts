import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_SYS,
    IMPORT_BRICKLET
} from "./util/definitions";
import { SET_SOLID_STATE_RELAY_FUNCTION } from "./util/function-declarations";

export function set_solid_state_relay(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const relayStatus = <string>block.getFieldValue("STATUS");

    // add definitions to generator
    Object.assign(generator.definitions_, {
        CONFIGURE_LOGGING,
        IMPORT_LOGGING,
        IMPORT_SYS,
        IMPORT_BRICKLET
    });

    // declare the 'play_audio_from_speech'-function
    const functionName = generator.provideFunction_(
        "set_solid_state_relay",
        SET_SOLID_STATE_RELAY_FUNCTION(generator),
    );

    return `${functionName}("${relayStatus}")\n`;
}

export {pythonGenerator};