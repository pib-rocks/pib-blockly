import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {IMPORT_BLOBCONVERTER, IMPORT_CV2, IMPORT_DEPTHAI, IMPORT_LOGGING, IMPORT_NUMPY, IMPORT_TIME } from "./util/definitions";
import { FACE_DETECTOR_CLASS } from "./util/function-declarations";

export function face_detector_start_stop(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const dropDownSetting = block.getFieldValue("SETTING");

    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_NUMPY,
        IMPORT_CV2,
        IMPORT_LOGGING,
        IMPORT_DEPTHAI,
        IMPORT_BLOBCONVERTER,
        IMPORT_TIME,
    });

    // declare the 'FaceDetector'-class
    const className = generator.provideFunction_(
        "FaceDetector",
        FACE_DETECTOR_CLASS(generator)
    );

    // generate code
    return dropDownSetting === "START"
        ? `fd = ${className}()\nlogging.info("Starting face detector")\n`
        : `fd.device.close()\nlogging.info("Closing face detector")\n`;
}

export function face_detector_running(
    block: Block, 
    generator: typeof pythonGenerator
) {
    // extract block-input
    const centerX = generator.getVariableName(
        block.getFieldValue("HORIZ_CENTER"),
    );
    const centerY = generator.getVariableName(
        block.getFieldValue("VERT_CENTER"),
    );

    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_NUMPY,
        IMPORT_CV2,
        IMPORT_DEPTHAI,
        IMPORT_BLOBCONVERTER,
        IMPORT_TIME,
    });

    // generate code
    return [
        `${centerX}, ${centerY} = fd.updateDetector()`,
        `if cv2.waitKey(1) == ord('q')`,
        `${generator.INDENT}break\n`
    ].join("\n");
}

export {pythonGenerator};
