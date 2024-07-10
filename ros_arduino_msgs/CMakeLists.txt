cmake_minimum_required(VERSION 3.5)
project(ros_arduino_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
        msg/AnalogFloat.msg
        msg/Analog.msg
        msg/ArduinoConstants.msg
        msg/Digital.msg
        msg/SensorState.msg
        )

set(srv_files
        srv/AnalogWrite.srv
        srv/AnalogSensorWrite.srv
        srv/AnalogFloatSensorWrite.srv
        srv/AnalogPinMode.srv           
        srv/AnalogRead.srv
        srv/AnalogSensorRead.srv
        srv/AnalogFloatSensorRead.srv
        srv/DigitalPinMode.srv
        srv/DigitalRead.srv
        srv/DigitalSensorRead.srv
        srv/DigitalSetDirection.srv
        srv/DigitalSensorPinMode.srv
        srv/DigitalWrite.srv
        srv/DigitalSensorWrite.srv
        srv/Enable.srv
        srv/Relax.srv
        srv/ServoAttach.srv
        srv/ServoDetach.srv
        srv/ServoRead.srv
        srv/ServoWrite.srv
        srv/SetSpeed.srv
        srv/SetServoSpeed.srv
        srv/UpdatePID.srv
        )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()