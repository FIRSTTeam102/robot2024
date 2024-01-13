package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

/** swerve module hardware abstraction interface */
public interface SwerveModuleIO {
	@AutoLog
	public static class SwerveModuleIOInputs {
		// double drivePosition_rad = 0.0;
		public double driveDistance_m = 0.0;
		public double driveVelocity_mps = 0.0;
		public double driveAppliedPercentage = 0.0;
		public double driveVoltage_V = 0.0;
		public double driveCurrent_A = 0.0;
		public double driveTemperature_C = 0.0;

		public double angleAbsolutePosition_rad = 0.0;
		public double angleVelocity_radps = 0.0;
		public double angleAppliedPercentage = 0.0;
		public double angleVoltage_V = 0.0;
		public double angleCurrent_A = 0.0;
		public double angleTemperature_C = 0.0;
	}

	/** updates the set of inputs */
	public default void updateInputs(SwerveModuleIOInputs inputs) {}

	/** runs the drive motor at the specified percentage of full power */
	public default void setDriveVoltage(double voltage) {}

	/** runs the drive motor at the specified velocity */
	public default void setDriveVelocity(double velocity) {}

	/** runs the angle motor at the specified voltage */
	public default void setAngleVoltage(double voltage) {}

	/** runs the angle motor to the specified position */
	public default void setAnglePosition(Rotation2d angle) {}

	/** enable or disable brake mode on the drive motor */
	public default void setDriveBrakeMode(boolean enable) {}

	/** set the offset (zero/forwards) of the angle position */
	public default void setOffset(double offset_rad) {}

	public default double getDriveCharacterizationVelocity_radps() {
		return 0.0;
	}

	/** called during testing to update motor pid config */
	public default void tunablePeriodic() {}
}
