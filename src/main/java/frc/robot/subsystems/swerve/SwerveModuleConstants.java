package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public record SwerveModuleConstants(
	int driveMotorId,
	int angleMotorId,
	double angleOffset_rad) {
	public double angleOffset_deg() {
		return Math.toDegrees(this.angleOffset_rad);
	}

	public Rotation2d angleOffset_2d() {
		return Rotation2d.fromRadians(this.angleOffset_rad);
	}
}