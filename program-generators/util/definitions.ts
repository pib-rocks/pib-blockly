// imports

export const IMPORT_RCLPY = "import rclpy";
export const IMPORT_NUMPY = "import numpy as np";
export const IMPORT_CV2 = "import cv2";
export const IMPORT_DEPTHAI = "import depthai as dai";
export const IMPORT_BLOBCONVERTER = "import blobconverter";
export const IMPORT_SYS = "import sys";
export const IMPORT_TIME = "import time";
export const IMPORT_LOGGING = "import logging";
export const IMPORT_PLAY_AUDIO_FROM_SPREECH =
    "from datatypes.srv import PlayAudioFromSpeech";
export const IMPORT_APPLY_JOINT_TRAJECTORY =
    "from datatypes.srv import ApplyJointTrajectory";
export const IMPORT_JOINT_TRAJECTORY_MESSAGES =
    "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint";
export const IMPORT_APPLY_POSE =
    "from datatypes.srv import ApplyPose";

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

//pose

export const INIT_APPLY_POSE_CLIENT = `
apply_pose_client = node.create_client(
    ApplyPose,
    'apply_pose'
)

logging.info(f"waiting for 'apply_pose' service to become available...")
apply_pose_client.wait_for_service()
logging.info(f"service now available")
`;