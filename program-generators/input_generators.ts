import {Block} from "blockly/core/block";

import {Order, pythonGenerator} from "blockly/python";

export function inputGenerator(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const prompt = generator.valueToCode(block, "TEXT", Order.ATOMIC);
    const type = <string>block.getFieldValue("TYPE");

    // generate code
    let code = "";
    switch (type) {
        case 'TEXT':
            code = `input(${prompt} + ': \\n')\n`;
            break;
        case 'NUMBER':
            code = `float(input(${prompt} + ': \\n'))\n`;
            break;
        default:
            throw new Error(`'${type}' is not a valid input-type.`);
    }
    return [code, Order.ATOMIC];
}

export {pythonGenerator};
