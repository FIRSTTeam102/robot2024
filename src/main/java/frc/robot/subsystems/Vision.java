// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DON'T LOOK AT THIS
package frc.robot.subsystems;

import frc.robot.io.FieldVisionIO;
import frc.robot.io.FieldVisionIOInputsAutoLogged;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	private FieldVisionIO fieldIO = new FieldVisionIO();
	public FieldVisionIOInputsAutoLogged fieldInputs = new FieldVisionIOInputsAutoLogged();

	public Vision() {}

	@Override
	public void periodic() {
		fieldIO.updateInputs(fieldInputs);
		Logger.processInputs(getName() + "/field", fieldInputs);
	}
}
