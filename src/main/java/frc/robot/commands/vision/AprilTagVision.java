// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class AprilTagVision extends Command {
	/** Creates a new AprilTagVision. */
	private static Vision vision;
	private static Swerve swerve;
	private boolean isAligned;
	private double robotRotate_radps;
	Optional<Alliance> alliance = DriverStation.getAlliance();
	public static double rotationalOffset = Math.atan2(VisionConstants.limelightXOffset, vision.find2dDistance()); // inverse
																																																									// tangent
																																																									// of
																																																									// X/y

	public AprilTagVision(Vision vision, Swerve swerve) {
		addRequirements(swerve);
		AprilTagVision.vision = vision; // VScode was throwing me an error. Using AprilTagVision class specifier was the fix
		AprilTagVision.swerve = swerve;
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
		robotRotate_radps = 0;
		isAligned = false;

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if ((vision.fieldInputs.targetAprilTag == VisionConstants.blueSpeakerTag) && (alliance.get() == Alliance.Blue)
			|| (vision.fieldInputs.targetAprilTag == VisionConstants.redSpeakerTag) && (alliance.get() == Alliance.Red)) {
			isAligned = (vision.pieceInputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairGamePieceBoundRotateX_rad)
				&& (vision.fieldInputs.crosshairToTargetErrorX_rad > -VisionConstants.crosshairGamePieceBoundRotateX_rad);
		}
		Logger.recordOutput("Fieldvision/isAligned", isAligned);

		if (vision.fieldInputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairGamePieceBoundRotateX_rad) {
			robotRotate_radps = VisionConstants.AprilTagRotateKp
				* vision.fieldInputs.crosshairToTargetErrorX_rad // why do we use Proportional limit * error - derivative
				- VisionConstants.AprilTagRotateKd;
		} else if (VisionConstants.crosshairGamePieceBoundRotateX_rad < vision.fieldInputs.crosshairToTargetErrorX_rad) {
			robotRotate_radps = VisionConstants.AprilTagRotateKp
				* vision.fieldInputs.crosshairToTargetErrorX_rad
				+ VisionConstants.AprilTagRotateKd;
		}
		robotRotate_radps *= -1; // Rotate opposite of error

		System.out.println("Swerve --> Shooting Speaker");
		swerve.drive(new Translation2d(0, 0), robotRotate_radps - rotationalOffset, false);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
