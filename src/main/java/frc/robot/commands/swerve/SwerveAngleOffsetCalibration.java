package frc.robot.commands.swerve;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

public class SwerveAngleOffsetCalibration extends Command {
	private Swerve swerve;

	private double[] sampledAngles = new double[4]; // sum of angles
	private double[] sendableAngles = sampledAngles; // average angles
	private int samples = 0;

	public SwerveAngleOffsetCalibration(Swerve swerve) {
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		swerve.resetModuleOffsets();

		sampledAngles = new double[] {0, 0, 0, 0};
		sendableAngles = new double[] {0, 0, 0, 0};
		samples = 0;
	}

	@Override
	public void execute() {
		samples++;
		var states = swerve.getStates();
		for (int i = 0; i < 4; i++) {
			sampledAngles[i] += states[i].angle.getRadians();
			sendableAngles[i] = sampledAngles[i] / samples;
		}
		Logger.recordOutput("SwerveAngleOffsetCalibration", sendableAngles);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}