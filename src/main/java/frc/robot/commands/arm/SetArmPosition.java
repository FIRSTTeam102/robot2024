// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class SetArmPosition extends Command {
	private Arm arm;
	private double targetPosition_deg; // did I do this right?

	private int armPIDSlot;

	/**
	 * Command to set the arm position, cancels the command and moves on 
	 * after being within the range given by {@link ArmConstants#accuracyTolerance_deg}
	 * @param arm Arm subsystem
	 * @param targetPosition_deg Position in degrees to set the arm to (0 is horizontal to robot base)
	 */
	public SetArmPosition(Arm arm, double targetPosition_deg) {
		this.arm = arm;
		this.targetPosition_deg = targetPosition_deg;
		addRequirements(arm);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		// only use the slow PID loop if we're in auto. Let it shake so teleop is quicker
		armPIDSlot = DriverStation.isAutonomous() ? arm.getBestPIDSlot(targetPosition_deg) : 0;
		arm.setPosition(targetPosition_deg, armPIDSlot);
	}

	@Override
	public void end(boolean interrupted) {
		// only stop if interrupted. Otherwise, continue adjusting arm position just move on to another command
		if (interrupted || targetPosition_deg >= 65) {
			arm.stop();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return arm.closeEnough();
	}
}
