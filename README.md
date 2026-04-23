# stormfin
underactuated fish finder

![prototyping](1000007958.jpg)

## microcontroller board
Arduino Due, 84MHz 32-bit ARM Cortex-M3 processor, 96KB SRAM
Adafruit METRO 328 for simple prototyping

## bill of materials
    1. SP17 IP68 10 pin waterproof connectors
    2. A2212 930KV brushless motors
    3. 55mID 60mmOD 4 blade propellers
    4. DFRobot IP68 6m UART ultrasonic sensor
    5. BNO055 IMU sensor
    6. Arduino Due
    7. 3S LiPo Battery (11.1V) with charger
    8. GLONASS + GPS PA1616D - 99 channel w/ 10Hz

## software overview
```mermaid
classDiagram
    Controller ..> Filter
    Controller ..> Sensors
    Controller ..> Thrusters
    Controller ..> Collocation
    Filter ..> Utilities
    Filter ..> LaminarModel
    Model <|-- LaminarModel
    Filter : +void initialize_state()
    Filter : +void process(double, double*, double*, double*, double*)
    Filter : +void estimate_measurements(double*, double*)
    Filter : +void update(double*, double*, double*)
    Sensors : +void qrot_pure(double*, double*)
    Sensors : +void body_to_nav(double*, double*, double*)
    Sensors : +void set_qrot(double*)
    Sensors : +void qrot_pure(double*, double*)
    Sensors : +void ultrasonic_distance(double)
    Thrusters : +void thrust_to_pwm(double*, int&)
    Collocation : +void optimal_thrust(double*, double*, int, double*)
```

## test plan
    1. prototype
    2. submersble (no power)
    3. pool test
    4. lake test (laminar no current)
    5. river test (laminar current)

## TODO
    1. insufficient memory for iostream, stdlib explicitly included
    2. use arduino BasicLinearAlgebra (currently prototyping using GSL)
    3. use DMA (Direct Memory Access) for sensor inputs rather than analogRead()
