package frc.robot.util;

import frc.robot.Robot;

/**
 * Various math-related methods that were not defined in other math libraries
 */
public class Math102 {
	public static final double falconCountsPerRotation = 2048.0;
	public static final double cancoderCountsPerRotation = 4096.0;
	public static final double twoPi = 2 * Math.PI;

	public static double angleModulus2pi(double angle) {
		return ((angle % twoPi) + twoPi) % twoPi;
	}

	/**
	 * Truncate a given double to a given number of decimal places
	 * @param num number to be truncated
	 * @param precision number of digits after the decimal point to truncate to
	 * @return Truncated double
	 */
	public static double truncate(double num, int precision) {
		return Math.floor(num * Math.pow(10, precision)) / Math.pow(10, precision);
	}

	public static double mirrorAngle_deg(double angle_deg) {
		return 180 - angle_deg;
	}

	public static double mirrorAngleByAlliance_deg(double angle_deg) {
		return Robot.isBlue() ? angle_deg : mirrorAngle_deg(angle_deg);
	}
}