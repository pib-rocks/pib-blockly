export const RESERVED_WORDS = [
    // common
    "rclpy",
    "node",
    "time",
    "sys",
    "np",
    "dai",
    "logging",
    "blobconverter",
    "stdout_handler",
    "stderr_handler",
    // play-audio-from-speech
    "PlayAudioFromSpeech",
    "play_audio_from_speech_client",
    // motor
    "ApplyJointTrajectory",
    "JointTrajectory",
    "JointTrajectoryPoint",
    "apply_joint_trajectory_client",
    "motor_name_to_position",
    // face detector
    "fd",
    // input
    "input_queue",
    "mpid",
    "program_prompt_publisher",
    "program_input_subscription"
].join(",");
