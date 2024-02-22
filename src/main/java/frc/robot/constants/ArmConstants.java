package frc.robot.constants;

public final class ArmConstants {
	public static final int leadMotorId = 29;
	public static final int followerMotorId = 30;

	public static final double shaftEncoderOffset_deg = 0.0; // needs to be tuned

	// pid
	public static final double kP = 0;
	public static final double kI = 0;
	public static final double kD = 0;

	public static final double maxOutput = .5;
	public static final double minOutput = -maxOutput;

	// feedforward
	public static final double kS = 0;
	public static final double kG = 0;
	public static final double kV = 0;
	public static final double kA = 0;

	public static final double verticalArmPos_deg = 90;

	// smartmotion
	public static double maxAccel_rpmps = 100;
	public static double maxVelocity_rpm = 50;

	// Conversion
	public static final int gearRatio = 1; // 1 IS PLACEHOLDER

	// closeEnough
	public static final double accuracyTolerance = 0.1;
}