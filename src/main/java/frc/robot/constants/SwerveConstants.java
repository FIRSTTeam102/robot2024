package frc.robot.constants;

import frc.robot.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;

import com.revrobotics.CANSparkBase.IdleMode;

public final class SwerveConstants {
	// FL, FR, BL, BR (matches AdvantageScope convention)
	public static final SwerveModuleConstants moduleConstants[] = {
		new SwerveModuleConstants(21, 22, 0),
		new SwerveModuleConstants(23, 24, 0),
		new SwerveModuleConstants(25, 26, 0),
		new SwerveModuleConstants(27, 28, 0)
	};

	// the left-to-right distance between the drivetrain wheels, should be measured from center to center
	public static final double trackWidth_m = 0.5;
	// the front-to-back distance between the drivetrain wheels, should be measured from center to center
	public static final double wheelBase_m = 0.5;

	// indexes must match moduleConstants
	public static final Translation2d[] moduleTranslations = {
		new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0),
	};

	/**
	 * MK4i module config
	 * @see https://github.com/SwerveDriveSpecialties/swerve-lib/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/SdsModuleConfigurations.java
	 */
	public static final double wheelDiameter_m = 0.10033;
	public static final double wheelRadius_m = wheelDiameter_m / 2;
	public static final double wheelCircumference_m = wheelDiameter_m * Math.PI;
	public static final double driveGearRatio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
	public static final boolean driveInverted = true;
	public static final IdleMode driveIdleMode = IdleMode.kBrake;
	public static final double driveRampTime_s = 0.25;

	public static final double angleGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
	public static final boolean angleInverted = true;
	public static final IdleMode angleIdleMode = IdleMode.kCoast;
	public static final double angleRampTime_s = 0.5;
	public static final boolean angleEncoderInverted = false;

	public static final double maxVelocity_mps = 5676 /* NEO max RPM */
		/ 60.0 / driveGearRatio * wheelCircumference_m;
	public static final double maxAngularVelocity_radps = 5676 /* NEO max RPM */
		/ 60.0 / angleGearRatio / Math.hypot(trackWidth_m / 2.0, wheelBase_m / 2.0);

	/* robot dimensions */
	public static final double robotfence = 0.35; // from botpose to the bumper

	/* drive motor PID values */
	public static final double driveKp = 0.04; // todo: tune for real
	public static final double driveKi = 0.0;
	public static final double driveKd = 0.0;
	public static final double driveKf = 1 / maxVelocity_mps; // https://github.com/OysterRiverOverdrive/Crescendo2024-Roomba/blob/main/src/main/java/frc/robot/Constants.java#L138
	/* sim drive motor PID values */
	public static final double simDriveKp = 0.003; // fixme: overshoots
	public static final double simDriveKi = 0.0;
	public static final double simDriveKd = 0.0;
	public static final double simDriveKf = 0.0;

	/* angle motor PID values */
	public static final double angleKp = 5.7; // todo: tune for real
	public static final double angleKi = 0.0;
	public static final double angleKd = 0.05;
	public static final double angleKf = 0.0;
	/* sim angle motor PID values */
	public static final double simAngleKp = 8.0;
	public static final double simAngleKi = 0.0;
	public static final double simAngleKd = 0.0;
	public static final double simAngleKf = 0.0;

	/* drive motor characterization (feed forward) */
	public static final double driveKs = 0.0092198;
	public static final double driveKv = 0.23218;
	/* sim drive motor characterization */
	public static final double simDriveKs = 0.117;
	public static final double simDriveKv = 0.133;

	/* angle motor characterization */
	// public static final double angleKs = 0;
	// public static final double angleKv = 0;
	// public static final double angleKa = 0;

	/* current limiting */
	public static final int driveCurrentLimit_amp = 50;
	public static final int angleCurrentLimit_amp = 25;
};
