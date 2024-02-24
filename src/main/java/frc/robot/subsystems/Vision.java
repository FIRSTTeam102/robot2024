package frc.robot.subsystems;

import frc.robot.io.FieldVisionIO;
import frc.robot.io.FieldVisionIOInputsAutoLogged;
import frc.robot.io.PieceVisionIO;
import frc.robot.io.PieceVisionIOInputsAutoLogged;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	private FieldVisionIO fieldIO = new FieldVisionIO();
	private PieceVisionIO pieceIO = new PieceVisionIO();
	public FieldVisionIOInputsAutoLogged fieldInputs = new FieldVisionIOInputsAutoLogged();
	public PieceVisionIOInputsAutoLogged pieceInputs = new PieceVisionIOInputsAutoLogged();

	public Vision() {}

	@Override
	public void periodic() {
		fieldIO.updateInputs(fieldInputs);
		pieceIO.updateInputs(pieceInputs);
		Logger.processInputs(getName() + "/field", fieldInputs);
		Logger.processInputs(getName() + "/piece", pieceInputs);
	}
}

/*
 * The point of this subsystem is to take all the inputs from VisionIO and log them.
 * From there we can take values from here and apply them elsewhere
 */