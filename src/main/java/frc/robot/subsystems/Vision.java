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

/*
 * The point of this subsystem is to take all the inputs from VisionIO and log them.
 * From there we can take values from here and apply them elsewhere
 */