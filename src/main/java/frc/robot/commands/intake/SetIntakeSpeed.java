// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakeSpeed extends Command {
	private double speed;
	private boolean isIndexing;
	private Intake intake;

	/**
	 * Create a command that sets the intake to a specified speed
	 * @param intake Intake Subsystem
	 * @param speed Speed to set the motor to, for -1 <= speed <= 1
	 * @param isIndexing if set to true, the command will ignore the note sensor.
	 */
	public SetIntakeSpeed(Intake intake, double speed, boolean isIndexing) {
		this.speed = speed;
		this.intake = intake;
		this.isIndexing = isIndexing;
		addRequirements(intake);
	}

	/**
	 * Set an intake to a standard speed given by {@link IntakeConstants#intakeSpeed intakeSpeed} if intaking and {@link IntakeConstants#indexSpeed indexSpeed} if indexing
	 * @param intake Intake Subsystem
	 * @param isIndexing if set to true, the command will ignore the note sensor. Default speed will also be slower.
	 */
	public SetIntakeSpeed(Intake intake, boolean isIndexing) {
		this(intake, isIndexing ? IntakeConstants.indexSpeed : IntakeConstants.intakeSpeed, isIndexing);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Run motor only if we are either indexing or don't detect a note. Or, if we are intaking (not indexing) and we
		// detect a note, don't run
		if (isIndexing || !intake.isHoldingNote())
			;
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
		// If we are intaking and we detect a note, cancel automatically. If we are indexing, the command will only be
		// canceled on interrupt
		return (!isIndexing && intake.isHoldingNote());
	}
}
