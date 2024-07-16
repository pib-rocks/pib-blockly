// imports

export const IMPORT_RCLPY = "import rclpy";
export const IMPORT_PUBLISHER = "from rclpy.publisher import Publisher";
export const IMPORT_NUMPY = "import numpy as np";
export const IMPORT_CV2 = "import cv2";
export const IMPORT_DEPTHAI = "import depthai as dai";
export const IMPORT_BLOBCONVERTER = "import blobconverter";
export const IMPORT_SYS = "import sys";
export const IMPORT_TIME = "import time";
export const IMPORT_LOGGING = "import logging";
export const IMPORT_QUEUE = "from queue import Empty, Queue";
export const IMPORT_PLAY_AUDIO_FROM_SPREECH =
    "from datatypes.srv import PlayAudioFromSpeech";
export const IMPORT_APPLY_JOINT_TRAJECTORY =
    "from datatypes.srv import ApplyJointTrajectory";
export const IMPORT_JOINT_TRAJECTORY_MESSAGES =
    "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint";
export const IMPORT_PROGRAM_PROMPT = "from datatypes.msg import ProgramPrompt";
export const IMPORT_PROGRAM_INPUT = "from datatypes.msg import ProgramInput";


// ros

export const INIT_ROS = `
rclpy.init()
node = rclpy.create_node("blockly_node")
`;

// logging

export const CONFIGURE_LOGGING = `
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

// play-audio-from-speech

export const INIT_PLAY_AUDIO_FROM_SPEECH_CLIENT = `
play_audio_from_speech_client = node.create_client(
    PlayAudioFromSpeech, 
    'play_audio_from_speech'
)

logging.info(f"waiting for 'play_audio_from_speech' service to become available...")
play_audio_from_speech_client.wait_for_service()
logging.info(f"service now available")
`;

// motor

export const INIT_APPLY_JOINT_TRASJECTORY_CLIENT = `
apply_joint_trajectory_client = node.create_client(
    ApplyJointTrajectory,
    'apply_joint_trajectory'
)

logging.info(f"waiting for 'apply_joint_trajectory' service to become available...")
apply_joint_trajectory_client.wait_for_service()
logging.info(f"service now available")
`;

export const INIT_MOTORNAME_TO_POSITION = `motor_name_to_position = {}`;

// input

export const GET_MPID = `
mpid = int(sys.argv[1])
`;

export const CREATE_INPUT_QUEUE = `
input_queue: Queue[str] = Queue()
`;

export const CREATE_PROMPT_PUBLISHER_AND_SUBSCRIBER = `
program_prompt_publisher: Publisher = node.create_publisher(
    ProgramPrompt, 
    "program_prompt",
    1
)

def on_input_received(program_input: ProgramInput) -> None:
    if program_input.mpid == mpid:
        if not program_input.is_present:
            logging.warn("user did not provide input, aborting...")
            exit(1)
        logging.info("received input from user")
        input_queue.put(program_input.input)


program_input_subscription = node.create_subscription(
    ProgramInput, 
    "program_input", 
    on_input_received
)
`;


