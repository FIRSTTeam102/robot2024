package frc.robot.io;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve;

public class GyroIOSim implements GyroIO {
	public GyroIOSim() {}

	// flag to set custom yaw on next loop
	private boolean setYawTriggered = false;
	private double setYaw_deg = 0;

	Swerve swerve;

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		if (swerve == null)
			swerve = RobotContainer.getInstance().swerve;
		if (swerve == null)
			return;

		if (setYawTriggered) {
			inputs.yaw_deg = setYaw_deg;
			setYawTriggered = false;
		}

		inputs.connected = true;
		double yaw_dps = Math
			.toDegrees(swerve.kinematics.toChassisSpeeds(swerve.moduleStates).omegaRadiansPerSecond);
		inputs.yaw_deg += yaw_dps * Constants.loopPeriod_s;
		// todo: sim other rotations if needed
	}

	@Override
	public void setYaw(double yaw_deg) {
		setYawTriggered = true;
		setYaw_deg = yaw_deg;
	}
}
