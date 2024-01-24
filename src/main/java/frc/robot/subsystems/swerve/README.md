Swerve library based on: (with modifications/refactors)
* [HuskieRobotics/3061-lib](https://github.com/HuskieRobotics/3061-lib)
* [Mechanical-Advantage/SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* [Team364/BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* [REVrobotics/MAXSwerve-Java-Template](https://github.com/REVrobotics/MAXSwerve-Java-Template)
* [Mechanical-Advantage/AdvantageKit/example_projects/advanced_swerve_drive](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive)

Motor communication is abstracted through [SwerveModuleIO](./SwerveModuleIO.java) and implementations for physical/sim hardware

# Poses
* the origin of the field is to the lower left corner (driver's right)
* zero is away from the driver
* angle increases in the CCW direction

# Rotations
* pitch = rotating around side-to-side axis
* yaw = rotating around vertical axis
* roll = rotating around front-to-back axis

# Spark MAX implementation
absolute angle encoder is a [thrifty encoder](https://www.thethriftybot.com/products/thrifty-absolute-magnetic-encoder) wired to the `ANA` pin of a [spark breakout board](https://www.revrobotics.com/rev-11-1278/) ([docs](https://docs.revrobotics.com/brushless/spark-max/specs/data-port#analog-input))