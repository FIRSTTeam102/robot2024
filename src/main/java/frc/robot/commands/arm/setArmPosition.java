// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class setArmPosition extends Command {
	private Arm arm;
	protected double targetPosition_rad; // did I do this right?

	/** Creates a new setArmPosition. */
	public setArmPosition(Arm arm, double targetPosition_rad) {
		this.arm = arm;
		this.targetPosition_rad = targetPosition_rad;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		arm.setPosition(targetPosition_rad);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		arm.stopArm();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (arm.closeEnough());
	}
}
