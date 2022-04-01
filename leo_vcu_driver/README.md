# leo_vcu_driver
`leo_vcu_driver` is the package to connect Autoware with LLC.

# Description

Purpose of this package is providing connection between Autoware Universe and Low Level Controller.

# Design

Design of package was made with respect to `Autoware.Universe` design architecture.

**Universe Flow diagram**

![Universe Flow diagram ](https://i.ibb.co/KjgQx87/universe-interface-design.png)

**leo_vcu_driver Flow diagram**

![leo_vcu_driver Flow diagram](https://i.ibb.co/BqJqbZj/Untitled-Diagram-2-drawio-1.png)

# Parameters


| Name                              | Type   | Description                                  |
|-----------------------------------|--------|----------------------------------------------|
| `base_frame_id`                   | string | frame id, default [base_link]                |
| `command_timeout_ms`              | double | timeout [ms]                                 |
| `loop_rate`                       | double | loop rate to publish commands to LLC [Hz]    |
| `reverse_gear_enabled_`           | bool   | enable reverse gear (True means Enabled)     |
| `emergency_stop_acceleration`     | float  | Acceleration parameter for emergency [m/s^2] |
| `gear_shift_velocity_threshold`   | float  | The maximum velocity to change gear [m/s]    |

# Future Improvements

* Add steering limits and parameters.