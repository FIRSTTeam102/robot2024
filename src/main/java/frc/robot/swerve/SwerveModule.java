package frc.robot.swerve;

import static frc.robot.constants.SwerveConstants.maxVelocity_mps;

import frc.robot.constants.Constants;
import frc.robot.util.Conversions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	public int moduleNumber;
	private Rotation2d lastAngle;
	public final SwerveModuleIO io;
	public final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

	public SwerveModule(int moduleNumber, SwerveModuleIO io) {
		this.moduleNumber = moduleNumber;
		this.io = io;

		lastAngle = getState().angle;
	}

	private SwerveModuleState optimizedState = new SwerveModuleState();
	private SwerveModuleState desiredState = new SwerveModuleState();
	private boolean isOpenLoop = false;
	private boolean forceAngle = false;

	/**
	 * Set this swerve module to the specified speed and angle.
	 *
	 * @param desiredState the desired state of the module
	 * @param isOpenLoop if true, the drive motor will be set to the calculated fraction of the max
	 * 	velocity; if false, the drive motor will set to the specified velocity using a closed-loop
	 * 	controller (PID)
	 * @param forceAngle if true, the module will be forced to rotate to the specified angle; if
	 * 	false, the module will not rotate if the velocity is too low
	 */
	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {
		this.desiredState = desiredState;
		this.isOpenLoop = isOpenLoop;
		this.forceAngle = forceAngle;
	}

	private boolean driveCharacterizationRunning = false;

	/** set the drive motor to the specified voltage, called repeatedly by SysId */
	public void runDriveCharacterization(double voltage) {
		setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)), true, true);
		io.setDriveVoltage(voltage);
		driveCharacterizationRunning = true;

		Logger.recordOutput("SysId/Swerve/Drive" + moduleNumber + "Target_V", voltage);
		Logger.recordOutput("SysId/Swerve/Drive" + moduleNumber + "Position_m", inputs.driveDistance_m);
		Logger.recordOutput("SysId/Swerve/Drive" + moduleNumber + "Velocity_mps", inputs.driveVelocity_mps);
	}

	private boolean angleCharacterizationRunning = false;

	/** set the angle motor to the specified voltage, called repeatedly by SysId */
	public void runAngleCharacterization(double voltage) {
		setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)), true, true);
		io.setAngleVoltage(voltage);
		angleCharacterizationRunning = true;
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			inputs.driveVelocity_mps,
			Rotation2d.fromRadians(inputs.angleAbsolutePosition_rad));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			inputs.driveDistance_m,
			Rotation2d.fromRadians(inputs.angleAbsolutePosition_rad));
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("SwerveModule " + moduleNumber, inputs);

		optimizedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(inputs.angleAbsolutePosition_rad));

		/*
		 * Unless the angle is forced (like X-stance), don't rotate if speed is too low.
		 * This prevents jittering if the controller isn't tuned perfectly.
		 * It also allows for smooth repeated movement as the wheel direction doesn't reset during pauses.
		 */
		var angle = (!forceAngle && Math.abs(optimizedState.speedMetersPerSecond) <= (maxVelocity_mps * 0.05))
			? lastAngle
			: optimizedState.angle;

		// run turn
		if (!angleCharacterizationRunning)
			io.setAnglePosition(angle);

		// update velocity based on angle error
		optimizedState.speedMetersPerSecond *= Math.cos(Math.abs(
			optimizedState.angle.getRadians() - inputs.angleAbsolutePosition_rad));

		Logger.recordOutput("SwerveModule " + moduleNumber + "/targetSpeed_mps", optimizedState.speedMetersPerSecond);

		// run drive
		if (driveCharacterizationRunning) {
			// don't try to change the state
		} else if (isOpenLoop) {
			double percentOutput = optimizedState.speedMetersPerSecond / maxVelocity_mps;
			io.setDriveVoltage(percentOutput * 12);
		} else {
			io.setDriveVelocity(optimizedState.speedMetersPerSecond);
		}

		lastAngle = angle;
		Logger.recordOutput("SwerveModule " + moduleNumber + "/targetAngle_rad",
			Conversions.angleModulus2pi(angle.getRadians()));

		if (Constants.tuningMode)
			io.tunablePeriodic();
	}

	public void setDriveBrakeMode(boolean enable) {
		io.setDriveBrakeMode(enable);
	}
}