// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.constants.Constants.OperatorConstants;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class ControllerUtil {
	/**
	 * Applies a deadband defined by {@link OperatorConstants#xboxStickDeadband xboxStickDeadband}, 
	 * and scales the resulting value on the function x^2 * sgn(x) (squares it but preserves the curve)
	 * @param value Unfiltered controller axis value, should be on the interval [-1.0, 1.0]
	 * @return Returns the squared and deadbanded controller axis
	 */
	public static double scaleAxis(double value) {
		value = MathUtil.applyDeadband(value, OperatorConstants.xboxStickDeadband);
		return Math.copySign(value * value, value);
	}
}
