# Project Tree

This file records the intended project module tree. Physical source files stay flat under `project/code` to match the SEEKFREE IAR project convention. The IAR project uses virtual groups to show the same structure.

```text
project/code
|-- app
|   |-- app.c / app.h
|   |-- app_config.h
|   |-- app_types.h
|   |-- app_state.c / app_state.h
|   |-- app_safety.c / app_safety.h
|   `-- app_scheduler.c / app_scheduler.h
|
|-- sensor
|   |-- sensor_imu.c / sensor_imu.h
|   |-- sensor_camera.c / sensor_camera.h
|   |-- sensor_encoder.c / sensor_encoder.h
|   `-- lsm6dsv16x_driver.c / lsm6dsv16x_driver.h
|
|-- estimate_perception
|   |-- estimator.c / estimator.h
|   `-- perception.c / perception.h
|
|-- control
|   |-- control_chassis.c / control_chassis.h
|   |-- control_leg.c / control_leg.h
|   `-- leg_config.c / leg_config.h
|
|-- actuator
|   |-- actuator_motor.c / actuator_motor.h
|   `-- actuator_servo.c / actuator_servo.h
|
`-- telemetry
    `-- telemetry.c / telemetry.h
```

## Module Boundaries

- `sensor_*`: collect hardware data and publish timestamped snapshots.
- `estimator`: combine IMU and wheel feedback into vehicle state.
- `perception`: convert camera frames into path/vision features.
- `control_*`: read snapshots and generate motor/servo commands.
- `actuator_*`: apply limits and write final hardware outputs.
- `app_*`: own state machine, safety policy, timing, and composition.
- `telemetry`: low-priority debug output only.
