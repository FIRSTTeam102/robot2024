package frc.robot.swerve;

import static frc.robot.constants.Constants.loopPeriod_s;
import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.Conversions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * A simulated version of the SwerveModuleIO interface.
 *
 * The swerve module is simulated as a flywheel connected to the drive motor and another flywheel
 * connected to the angle motor.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
	private FlywheelSim driveWheelSim = new FlywheelSim(DCMotor.getFalcon500(1), driveGearRatio, 0.025);
	private FlywheelSim angleWheelSim = new FlywheelSim(DCMotor.getNEO(1), angleGearRatio, 0.004);

	private double angleAbsolutePosition_rad = 0.0; // Math.random() * Conversions.twoPi;
	private double driveApplied_V = 0.0;
	private double angleApplied_V = 0.0;
	private boolean isDriveOpenLoop = true;
	private double driveSetpoint_mps = 0.0;

	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(simDriveKs, simDriveKv, simDriveKa);
	private PIDController driveController = new PIDController(simDriveKp, simDriveKi, simDriveKd);

	private PIDController angleController = new PIDController(simAngleKp, simDriveKi, simDriveKd);
	private boolean isAngleOpenLoop = true;

	public SwerveModuleIOSim() {
		angleController.enableContinuousInput(0, Conversions.twoPi);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		if (!isAngleOpenLoop) {
			angleWheelSim.setInputVoltage(angleController.calculate(inputs.angleAbsolutePosition_rad));
		}

		// update the models
		driveWheelSim.update(loopPeriod_s);
		angleWheelSim.update(loopPeriod_s);

		// update the inputs that will be logged
		double angleDiff_rad = angleWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s;
		inputs.anglePosition_rad += angleDiff_rad;
		// angleAbsolutePosition_rad += angleDiff_rad;
		angleAbsolutePosition_rad = Conversions.angleModulus2pi(angleAbsolutePosition_rad + angleDiff_rad);
		inputs.angleAbsolutePosition_rad = angleAbsolutePosition_rad;

		// inputs.drivePosition_rad = inputs.drivePosition_rad
		// + driveWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s;
		inputs.driveDistance_m += (driveWheelSim.getAngularVelocityRadPerSec() * loopPeriod_s * wheelRadius_m);
		inputs.driveVelocity_mps = driveWheelSim.getAngularVelocityRadPerSec() * wheelRadius_m;
		inputs.driveVoltage_V = driveApplied_V;
		inputs.driveCurrent_A = Math.abs(driveWheelSim.getCurrentDrawAmps());

		inputs.angleVelocity_radps = angleWheelSim.getAngularVelocityRadPerSec();
		inputs.angleVoltage_V = angleApplied_V;
		inputs.angleCurrent_A = Math.abs(angleWheelSim.getCurrentDrawAmps());

		if (!isDriveOpenLoop) {
			double velocity_radps = driveSetpoint_mps / wheelRadius_m;
			driveApplied_V = driveFeedforward.calculate(velocity_radps)
				+ driveController.calculate(inputs.driveVelocity_mps, velocity_radps);
			driveApplied_V = MathUtil.clamp(driveApplied_V, -12.0, 12.0);
			driveWheelSim.setInputVoltage(driveApplied_V);
		}

		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(driveWheelSim.getCurrentDrawAmps(),
				angleWheelSim.getCurrentDrawAmps()));
	}

	@Override
	public void setDriveVoltage(double voltage) {
		isDriveOpenLoop = true;
		driveController.reset();
		driveApplied_V = MathUtil.clamp(voltage, -12.0, 12.0);
		driveWheelSim.setInputVoltage(driveApplied_V);
	}

	@Override
	public void setDriveVelocity(double velocity) {
		isDriveOpenLoop = false;
		driveSetpoint_mps = velocity;
	}

	@Override
	public void setAngleVoltage(double voltage) {
		isAngleOpenLoop = true;
		angleController.reset();
		angleApplied_V = MathUtil.clamp(voltage, -12.0, 12.0);
		angleWheelSim.setInputVoltage(angleApplied_V);
	}

	@Override
	public void setAnglePosition(Rotation2d angle) {
		isAngleOpenLoop = false;
		angleController.setSetpoint(angle.getRadians());
	}

	@Override
	public double getDriveCharacterizationVelocity_radps() {
		return driveWheelSim.getAngularVelocityRadPerSec();
	}
}
