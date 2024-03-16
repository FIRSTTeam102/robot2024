// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

public class GamePieceVision extends Command {
	private Vision vision;
	private Swerve swerve;
	private double robotRotate_radps;
	private boolean isAligned;
	private PIDController pidController = new PIDController(VisionConstants.visionRotateKp, 0,
		VisionConstants.visionRotateKd);

	/** Creates a new GamePieceVision. */
	public GamePieceVision(Vision vision, Swerve swerve) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(swerve);
		this.vision = vision;
		this.swerve = swerve;

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		robotRotate_radps = 0;
		isAligned = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		isAligned = (vision.pieceInputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairGamePieceBoundRotateX_rad)
			&& (vision.pieceInputs.crosshairToTargetErrorX_rad > -VisionConstants.crosshairGamePieceBoundRotateX_rad);
		Logger.recordOutput("GamePieceVision/isAligned", isAligned);

		// When we see a ground GamePiece, we will rotate to it
		// if (vision.pieceInputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairGamePieceBoundRotateX_rad) {
		// robotRotate_radps = VisionConstants.gamePieceRotateKp
		// * vision.pieceInputs.crosshairToTargetErrorX_rad
		// - VisionConstants.gamePieceRotateKd;
		// } else if (VisionConstants.crosshairGamePieceBoundRotateX_rad < vision.pieceInputs.crosshairToTargetErrorX_rad) {
		// robotRotate_radps = VisionConstants.gamePieceRotateKp
		// * vision.pieceInputs.crosshairToTargetErrorX_rad
		// + VisionConstants.gamePieceRotateKd;
		// }
		// robotRotate_radps *= -1; // Rotate opposite of error
		if (!MathUtil.isNear(0, vision.pieceInputs.crosshairToTargetErrorX_rad,
			VisionConstants.crosshairGamePieceBoundRotateX_rad)) {
			robotRotate_radps = pidController.calculate(vision.pieceInputs.crosshairToTargetErrorX_rad, 0);
		} else {
			robotRotate_radps = 0;
		}
		// Generate a continuously updated rotation to GamePiece
		System.out.println("Swerve --> GamePiece");
		Logger.recordOutput("Vision/targetRotation", robotRotate_radps);
		swerve.drive(new Translation2d(0, 0), robotRotate_radps, false); // TELL DRIVE THAT THEY ARE ALWAYS ROBOT ORIENTED
																																			// WHEN ALIGN TO NOTE
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
