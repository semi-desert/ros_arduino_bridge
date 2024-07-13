from rclpy.impl import rcutils_logger
rc_logger = rcutils_logger.RcutilsLogger(name="rc_logger")


def to_bytes(ins):
    return bytes(ins, 'utf-8')


def to_str(inb):
    return str(inb, 'utf-8')