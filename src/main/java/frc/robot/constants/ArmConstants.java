package frc.robot.constants;

public final class ArmConstants {
	public static final int leadMotorId = 29;
	public static final int followerMotorId = 30;

	public static final double shaftEncoderOffset_deg = 125.28; // needs to be tuned

	// pid
	public static final double kP = .000089;
	public static final double kI = 0;
	public static final double kD = .00000185;

	public static final double maxOutput = .5;
	public static final double minOutput = -maxOutput;

	public static final double manualMaxOutput = .4;

	// feedforward
	public static final double kS = 0;
	public static final double kG = .5;
	public static final double kV = 0;
	public static final double kA = 0;

	public static final double verticalArmPos_deg = 90;

	// smartmotion
	public static double maxAccel_rpmps = 7200;
	public static double maxVelocity_rpm = 3800;

	// Conversion (not needed)
	// public static final int gearRatio = 1; // 1 IS PLACEHOLDER

	// closeEnough
	public static final double accuracyTolerance_deg = .65;
}