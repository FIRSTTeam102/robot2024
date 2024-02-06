// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakeSpeed extends Command {
	private double speed;
	private Intake intake;

	/**
	 * Create a command that sets the intake to a specified speed
	 * @param intake Intake Subsystem
	 * @param speed Speed to set the motor to, for -1 <= speed <= 1
	 */
	public SetIntakeSpeed(Intake intake, double speed) {
		this.speed = speed;
		this.intake = intake;
		addRequirements(intake);
	}

	/**
	 * Set an intake to a standard speed given by {@link IntakeConstants#maxMotorSpeed IntakeConstants.maxMotorSpeed}
	 * @param intake Intake Subsystem
	 */
	public SetIntakeSpeed(Intake intake) {
		this(intake, IntakeConstants.maxMotorSpeed);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		intake.setMotorSpeed(speed);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stopMotor();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return intake.detectNote();
	}
}
