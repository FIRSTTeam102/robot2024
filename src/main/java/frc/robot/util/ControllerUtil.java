// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.constants.Constants.OperatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Various utility functions that deal with controllers and human input
 */
public class ControllerUtil {
	/**
	 * Applies a deadband defined by {@link OperatorConstants#xboxStickDeadband}, 
	 * and scales the resulting value on the function x^2 * sgn(x) (squares it but preserves direction)
	 * @param value Unfiltered controller axis value, should be on the interval [-1.0, 1.0]
	 * @return Returns the squared and deadbanded controller axis
	 */
	public static double scaleAxis(double value) {
		value = MathUtil.applyDeadband(value, OperatorConstants.xboxStickDeadband);
		return Math.copySign(value * value, value);
	}

	/**
	* Create a command that, when scheduled, will send a 550 ms rumble pulse to the given controller.
	* @param controller
	* @return Pulse Rumble command
	*/
	public static Command pulseRumble(CommandXboxController controller) {
		return Commands.startEnd(() -> controller.getHID().setRumble(RumbleType.kBothRumble, .65),
			() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0), new Subsystem[] {}).withTimeout(.55);
	}
}
