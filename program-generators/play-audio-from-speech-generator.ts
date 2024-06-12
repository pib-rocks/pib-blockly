import {CodeGenerator} from "blockly";
import {Block} from "blockly/core/block";

import {Order, pythonGenerator} from "blockly/python";

const PLAY_AUDIO_FROM_SPEECH_DEF = `
def play_audio_from_speech(speech: str, voice: str) -> None:

    logging.info(f"received request to say '{speech}' as '{voice}'.")

    request = PlayAudioFromSpeech.Request()
    request.speech = speech
    request.join = True

    if voice == 'Hannah':
        request.gender = "Female"
        request.language = "German"
    elif voice == 'Daniel':
        request.gender = "Male"
        request.language = "German"
    elif voice == 'Emma':
        request.gender = "Female"
        request.language = "English"
    elif voice == 'Brian':
        request.gender = "Male"
        request.language = "English"
    else:
        logging.error(f"unrecognized voice: '{voice}', aborting...")
        return

    future = play_audio_from_speech_client.call_async(request)

    logging.info(f"now speaking...")
    rclpy.spin_until_future_complete(node, future)
    logging.info("finished speaking.")
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
play_audio_from_speech_client = node.create_client(
    PlayAudioFromSpeech, 
    'play_audio_from_speech'
)

logging.info(f"waiting for 'play_audio_from_speech' service to become available...")
play_audio_from_speech_client.wait_for_service()
logging.info(f"service now available")
`;

pythonGenerator.addReservedWords(
    "play_audio_from_speech_client,PlayAudioFromSpeechClient()",
);

export function playAudioFromSpeechGenerator(
    block: Block,
    generator: CodeGenerator,
) {

    // extract block-input
    const voiceName = <string>block.getFieldValue("VOICENAME");
    const textInput = generator.valueToCode(block, "TEXT_INPUT", Order.ATOMIC);

    // declare python imports
    (generator as any).definitions_["import_rclpy"] = "import rclpy";
    (generator as any).definitions_["import_sys"] = "import sys";
    (generator as any).definitions_["import_logging"] = "import logging";
    (generator as any).definitions_["import_ApplyPlayAudioFromSpeech"] = "from datatypes.srv import PlayAudioFromSpeech";

    // configure logging
    (generator as any).definitions_["configure_logging"] = CONFIGURE_LOGGING;
    
    // initialize rclpy and create node
    (generator as any).definitions_["init_ros"] = INIT_ROS;

    // initialize play-audio-from-speech-client
    (generator as any).definitions_["init_play_audio_from_speech_client"] = INIT_CLIENT;

    // declare the 'play_audio_from_speech'-function
    generator.provideFunction_(
        "play_audio_from_speech",
        PLAY_AUDIO_FROM_SPEECH_DEF,
    );

    return `play_audio_from_speech(${textInput}, ${voiceName})\n`;
}

export {pythonGenerator};
