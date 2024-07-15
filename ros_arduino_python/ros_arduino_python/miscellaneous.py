import json
from rclpy.impl import rcutils_logger
rc_logger = rcutils_logger.RcutilsLogger(name="rc_logger")
import time


def to_bytes(ins):
    return bytes(ins, 'utf-8')


def to_str(inb):
    return str(inb, 'utf-8')


def declare_params(self):
    self.declare_parameter('port', '/dev/ttyUSB0')
    self.declare_parameter('baud', 57600)
    self.declare_parameter('timeout', 0.1)
    self.declare_parameter('base_frame', 'base_footprint')
    self.declare_parameter('rate', 50)  # 50

    self.declare_parameter('sensorstate_rate', 10)  # 10
    self.declare_parameter('use_base_controller', True)
    self.declare_parameter('diagnotics_error_threshold', 10)
    self.declare_parameter('diagnotics_rate', 1.0)
    
    self.declare_parameter('joint_update_rate', 10)  # 10
    self.declare_parameter('base_controller_rate', 10)  # 10
    self.declare_parameter('base_controller_timeout', 1.0)
    self.declare_parameter('odom_linear_scale_correction', 1.0)
    self.declare_parameter('odom_angular_scale_correction', 1.0)
    self.declare_parameter('use_imu_heading', False)
    self.declare_parameter('publish_odom_base_transform', True)

    self.declare_parameter('wheel_diameter', 0.066)
    self.declare_parameter('wheel_track', 0.215)
    self.declare_parameter('encoder_resolution', 3960)
    self.declare_parameter('gear_reduction', 1.0)

    self.declare_parameter('Kp', 2)
    self.declare_parameter('Kd', 1)
    self.declare_parameter('Ki', 0)
    self.declare_parameter('Ko', 50)
    self.declare_parameter('accel_limit', 1.0)
    self.declare_parameter('motors_reversed', False)
    self.declare_parameter('detect_enc_jump_error', False)
    self.declare_parameter('enc_jump_error_threshold', 1000)
    self.declare_parameter('base_diagnotics_error_threshold', 10)
    self.declare_parameter('base_diagnotics_rate', 1.0)


def declare_json_params(self):
    self.declare_parameter('sensors', """{
                                "onboard_led": 
                                    {"pin": 13, "type": "Digital", "rate": 10, "direction": "output"}
                            }""")
    self.declare_parameter('joints', """{}""")
    #self.declare_parameter('joints', """{
    #                            "head_pan_joint": 
    #                                {"pin": 3, "init_position": 0, "init_speed": 90, "neutral": 90, "min_position": -90, "max_position": 90, "invert": false, "continuous": false},
    #                            "head_tilt_joint": 
    #                                {"pin": 5, "init_position": 0, "init_speed": 90, "neutral": 90, "min_position": -90, "max_position": 90, "invert": false, "continuous": false}
    #                       }""")
    self.declare_parameter('controllers', """{}""")
    #self.declare_parameter('controllers', """{
    #                        }""")
    self.sensors_config = load_json_config(self, "sensors")
    self.joints_config = load_json_config(self, "joints")
    self.controllers_config = load_json_config(self, "controllers")


def load_json_config(self, param_name):
    sensors_config_str = self.get_parameter(param_name).get_parameter_value().string_value
    try:
        config = json.loads(sensors_config_str)
        print("json config:", config)
        return config
    except Exception as e:
        self.get_logger().warn(f'Failed to load config: {e}')



class TimerError(Exception):
    """A custom exception used to report errors in use of Timer class"""

class Timer:
    def __init__(self):
        self._start_time = None

    def start(self):
        """Start a new timer"""
        if self._start_time is not None:
            raise TimerError(f"Timer is running. Use .stop() to stop it")

        self._start_time = time.perf_counter()

    def stop(self):
        """Stop the timer, and report the elapsed time"""
        if self._start_time is None:
            raise TimerError(f"Timer is not running. Use .start() to start it")

        elapsed_time = time.perf_counter() - self._start_time
        self._start_time = None
        print(f"Elapsed time: {elapsed_time:0.4f} seconds")