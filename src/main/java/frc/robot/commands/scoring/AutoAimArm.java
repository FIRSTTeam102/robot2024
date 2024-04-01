// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import frc.robot.constants.ScoringConstants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimArm extends Command {
	private Arm arm;
	private Shooter shooter;
	private Vision vision;

	public AutoAimArm(Arm arm, Shooter shooter, Vision vision) {
		addRequirements(arm);
		addRequirements(shooter);

		this.arm = arm;
		this.shooter = shooter;
		this.vision = vision;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		ScoringPosition currentPos = vision.estimateScoringPosition_math();
		arm.setPosition(currentPos.armAngle_deg());
		shooter.setVelocity(currentPos.shooterSpeed_rpm());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			shooter.stop();
			arm.stop();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return arm.closeEnough();
	}
}
