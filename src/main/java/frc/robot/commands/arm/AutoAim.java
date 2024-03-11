// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.constants.VisionConstants;
import frc.robot.io.FieldVisionIOInputsAutoLogged;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

public class AutoAim extends Command {
	private Shooter shooter;
	private Arm arm;
	private Vision vision; // needed just to set the priority tag to stare at
	private FieldVisionIOInputsAutoLogged fieldvisionio; // the data we take from
	Optional<Alliance> alliance = DriverStation.getAlliance();

	/** Creates a new AutoAim. */
	public AutoAim(Arm arm, Vision vision, Shooter shooter) {
		this.arm = arm;
		this.vision = vision;
		this.shooter = shooter;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (alliance.get() == Alliance.Blue) {
			vision.setPriorityId(VisionConstants.blueSpeakerTag);
		}
		if (alliance.get() == Alliance.Red) {
			vision.setPriorityId(VisionConstants.redSpeakerTag);
		}
	}

	@Override
	public void execute() {
		var scoringPositon = vision.estimateScoringPosition_map();
		arm.setPosition(scoringPositon.armAngle_deg());
		shooter.setVelocity(scoringPositon.shooterSpeed_rpm());
	}

	@Override
	public void end(boolean interrupted) {
		arm.setPosition(5);
		shooter.setVelocity(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
