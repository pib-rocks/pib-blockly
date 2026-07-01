// imports

export const IMPORT_RCLPY = "import rclpy";
export const IMPORT_NUMPY = "import numpy as np";
export const IMPORT_CV2 = "import cv2";
export const IMPORT_DEPTHAI = "import depthai as dai";
export const IMPORT_BLOBCONVERTER = "import blobconverter";
export const IMPORT_SYS = "import sys";
export const IMPORT_OS = "import os";
export const IMPORT_TIME = "import time";
export const IMPORT_LOGGING = "import logging";
export const IMPORT_PLAY_AUDIO_FROM_SPREECH =
    "from datatypes.srv import PlayAudioFromSpeech";
export const IMPORT_APPLY_JOINT_TRAJECTORY =
    "from datatypes.srv import ApplyJointTrajectory";
export const IMPORT_GET_JOINT_POSITION =
    "from datatypes.srv import GetJointPosition";
export const IMPORT_JOINT_TRAJECTORY_MESSAGES =
    "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint";
export const IMPORT_POSE_CLIENT = "from pib_api_client import pose_client";
export const IMPORT_SET_SOLID_STATE_RELAY =
    "from datatypes.srv import SetSolidStateRelay";
export const IMPORT_SOLID_STATE_RELAY_STATE =
    "from datatypes.msg import SolidStateRelayState";
export const IMPORT_PARAMIKO = "import paramiko";

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
    datefmt="%y-%m-%d %H:%M:%S", force=True
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

export const INIT_GET_JOINT_POSITION_CLIENT = `
get_joint_position_client = node.create_client(
    GetJointPosition,
    'get_joint_position'
)

logging.info(f"waiting for 'get_joint_position' service to become available...")
get_joint_position_client.wait_for_service()
logging.info(f"service now available")
`;

// set solid state relay

export const INIT_SET_SOLID_STATE_RELAY_STATE_CLIENT = `
set_solid_state_relay_state_client = node.create_client(
    SetSolidStateRelay,
    'set_solid_state_relay_state'
)

logging.info(f"waiting for 'set_solid_state_relay_state' service to become available...")
set_solid_state_relay_state_client.wait_for_service()
logging.info(f"service now available")
`;
