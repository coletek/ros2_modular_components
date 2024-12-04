# ros2_modular_components

Place these folders into ros_ws/src/ and use where needed. See generic_ui/ for some examples of usage.

An example ros_ws.yaml might be:

```
/device_lights:
  ros__parameters:
    count: 1
    pins: [ 5 ]
    freqs: [ 100 ]
    subs: [ /device_light ]
/device_fans:
  ros__parameters:
    count: 1
    pins: [ 13 ]
    freqs: [ 25000 ]
    subs: [ /device_fan ]
/device_buzzers:
  ros__parameters:
    mode: pi
    pin: 12
    sub: /device_buzzer
    srv: /device_buzzer/mode
/device_motors:
  ros__parameters:
    count: 6
    pins_pwm: [ 17, 19, 21, 23, 25, 27 ]
    pins_dir: [ 16, 18, 20, 22, 24, 26 ]
    freqs: [ 25000, 25000, 25000, 25000, 25000, 25000 ]
    speed_min: [ 180, 180, 160, 160, 160, 160 ]
    speed_max: [ 255, 255, 255, 255, 255, 255 ]
    subs: [ /device_motor/rotate_left, /device_motor/rotate_right, /device_motor/back_left, /device_motor/back_right, /device_motor/front_left, /device_motor/front_right ]
/device_max7301atl:
  ros__parameters:
    spi_bus: 0
    spi_device: 0
    spi_baud: 1000000
    ch_count: 9
    chs: [ 8, 12, 9, 13, 10, 14, 11, 15, 16 ]
    modes: [ output, output, output, output, output, output, output, output, output ]
    is_inverse_state: [ false, false, false, false, false, true, true, true, true ]    
    topics: [ /device_max730atl/air_compressor, /device_max7301atl/heater_bottom_left, /device_max7301atl/heater_bottom_right, /device_max7301atl/heater_top_left, /device_max7301atl/heater_top_right, /device_max7301atl/fan_bottom_left, /device_max7301atl/fan_bottom_right, /device_max7301atl/fan_top_left, /device_max7301atl/fan_top_right ]

/sensor_mcp3008:
  ros__parameters:
    spi_bus: 0
    spi_device: 1
    spi_baud: 1000000
    vref: 4.096
    bits: 1023
    ntc_beta: 3950.0
    ntc_resistance: 10000.0
    freq: 10.0
    ch_count: 7
    chs: [ 0, 1, 2, 3, 4, 5, 6 ]
    types: [ percentage, percentage, percentage, percentage, percentage, percentage, lm335dt ]
    pubs: [ /sensor_mcp3008/percentage_motor_rotate_left, /sensor_mcp3008/percentage_motor_rotate_right, /sensor_mcp3008/percentage_motor_back_left, /sensor_mcp3008/percentage_motor_back_right, /sensor_mcp3008/percentage_motor_front_left, /sensor_mcp3008/percentage_motor_front_right, /sensor_mcp3008/temperature_control_box ]
/sensor_onewire:
  ros__parameters:
    count: 8    
    ids: [ 3b-00000018370f, 3b-000000183730, 3b-0000001838b8, 3b-0000001838b6, 3b-000000183731, 3b-0000001838b4, 3b-0000001838bb, 3b-000000184195 ]
    types: [ temperature, temperature, temperature, temperature, temperature, temperature, temperature, temperature ]
    freqs: [ 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 ]
    pubs: [ /sensor_onewire/temperature_oven_top_right, /sensor_onewire/temperature_oven_bottom_front, /sensor_onewire/temperature_oven_top_back, /sensor_onewire/temperature_oven_bottom_back, /sensor_onewire/temperature_oven_top_left, /sensor_onewire/temperature_oven_bottom_right, /sensor_onewire/temperature_oven_top_front, /sensor_onewire/temperature_oven_bottom_left ]

/controller_dual_actuator_back:
  ros__parameters:
    p_gain: 10.0
    i_gain: 0.0
    d_gain: 0.0
    hysteresis: 0.005
    motor_deadband: 0.55
    freq_hz: 10.0
    sub: /controller_dual_actuator_back/set_point
    pub: /controller_dual_actuator_back/feedback
    subs_sensors: [ /sensor_mcp3008/percentage_motor_back_left, /sensor_mcp3008/percentage_motor_back_right ]
    pubs_motors: [ /device_motor/back_left, /device_motor/back_right ]
    srv: /controller_dual_actuator_back/enable
/controller_dual_actuator_front:
  ros__parameters:
    p_gain: 10.0
    i_gain: 0.0
    d_gain: 0.0
    hysteresis: 0.005
    motor_deadband: 0.55
    freq_hz: 10.0
    sub: /controller_dual_actuator_front/set_point
    pub: /controller_dual_actuator_front/feedback
    subs_sensors: [ /sensor_mcp3008/percentage_motor_front_left, /sensor_mcp3008/percentage_motor_front_right ]
    pubs_motors: [ /device_motor/front_left, /device_motor/front_right ]
    srv: /controller_dual_actuator_front/enable
/controller_dual_actuator_rotate:
  ros__parameters:
    p_gain: 10.0
    i_gain: 0.0
    d_gain: 0.0
    hysteresis: 0.005
    motor_deadband: 0.55
    freq_hz: 10.0
    sub: /controller_dual_actuator_rotate/set_point
    pub: /controller_dual_actuator_rotate/feedback
    subs_sensors: [ /sensor_mcp3008/percentage_motor_rotate_left, /sensor_mcp3008/percentage_motor_rotate_right ]
    pubs_motors: [ /device_motor/rotate_left, /device_motor/rotate_right ]
    srv: /controller_dual_actuator_rotate/enable
```
