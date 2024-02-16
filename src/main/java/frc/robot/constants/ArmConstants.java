package frc.robot.constants;

public final class ArmConstants {
	public static final int motorId = 2; // placeholder

	// pid
	public static final double kP = 0;
	public static final double kI = 0;
	public static final double kD = 0;

	public static final double minOutput = -1;
	public static final double maxOutput = 1;

	// feedforward
	public static final double kS = 0;
	public static final double kG = 0;
	public static final double kV = 0;
	public static final double kA = 0;

	public static final double verticalArmPos_deg = 90;

	// smartmotion (all values are for the motor, not the arm)
	public static double maxAccel_rpmps = 400;
	public static double maxVelocity_rpm = 800;

	// Conversion
	public static final int gearRatio = 1; // 1 IS PLACEHOLDER

	// closeEnough
	public static final double closeVar = 0.1;
}