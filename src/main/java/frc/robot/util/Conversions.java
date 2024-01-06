package frc.robot.util;

public class Conversions {
	public static final double falconCountsPerRotation = 2048.0;
	public static final double cancoderCountsPerRotation = 4096.0;
	public static final double twoPi = 2 * Math.PI;

	public static double angleModulus2pi(double angle) {
		return ((angle % twoPi) + twoPi) % twoPi;
	}
}