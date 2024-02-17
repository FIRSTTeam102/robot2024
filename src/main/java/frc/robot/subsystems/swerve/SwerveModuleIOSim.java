package frc.robot.subsystems.swerve;

import static frc.robot.constants.Constants.loopPeriod_s;
import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.Conversions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
	private DCMotorSim driveSim = new DCMotorSim(DCMotor.getFalcon500(1), driveGearRatio, 0.025);
	private DCMotorSim angleSim = new DCMotorSim(DCMotor.getNEO(1), angleGearRatio, 0.004);

	private double angleInitPosition_rad = 0.0; // Math.random() * Conversions.twoPi;

	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
	private PIDController driveController = new PIDController(0.1, 0.0, 0.0);
	private boolean driveClosedLoop = false;
	private double driveSetpoint_mps = 0.0;
	private double driveApplied_V = 0.0;

	private PIDController angleController = new PIDController(10.0, 0.0, 0.0);
	private boolean angleClosedLoop = false;
	private double angleSetpoint_rad = 0.0;
	private double angleApplied_V = 0.0;

	public SwerveModuleIOSim() {
		angleController.enableContinuousInput(0, Conversions.twoPi);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		if (angleClosedLoop) {
			angleApplied_V = MathUtil.clamp(angleController.calculate(inputs.angleAbsolutePosition_rad, angleSetpoint_rad),
				-12.0, 12.0);
			angleSim.setInputVoltage(angleApplied_V);
		}

		if (driveClosedLoop) {
			double velocity_radps = driveSetpoint_mps / wheelRadius_m;
			driveApplied_V = MathUtil.clamp(
				driveFeedforward.calculate(velocity_radps)
					+ driveController.calculate(inputs.driveVelocity_mps, velocity_radps),
				-12.0, 12.0);
			driveSim.setInputVoltage(driveApplied_V);
		}

		// update the models
		driveSim.update(loopPeriod_s);
		angleSim.update(loopPeriod_s);

		// inputs.drivePosition_rad = driveSim.getAngularPositionRad();
		inputs.driveDistance_m = driveSim.getAngularPositionRad() * wheelRadius_m;
		inputs.driveVelocity_mps = driveSim.getAngularVelocityRadPerSec() * wheelRadius_m;
		inputs.driveVoltage_V = driveApplied_V;
		inputs.driveCurrent_A = Math.abs(driveSim.getCurrentDrawAmps());

		inputs.anglePosition_rad = angleSim.getAngularPositionRad();
		inputs.angleAbsolutePosition_rad = Conversions.angleModulus2pi(angleInitPosition_rad + inputs.anglePosition_rad);
		inputs.angleVelocity_radps = angleSim.getAngularVelocityRadPerSec();
		inputs.angleVoltage_V = angleApplied_V;
		inputs.angleCurrent_A = Math.abs(angleSim.getCurrentDrawAmps());

		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(inputs.driveCurrent_A, inputs.angleCurrent_A));
	}

	@Override
	public void setDriveVoltage(double voltage) {
		driveClosedLoop = false;
		driveController.reset();
		driveApplied_V = MathUtil.clamp(voltage, -12.0, 12.0);
		driveSim.setInputVoltage(driveApplied_V);
	}

	@Override
	public void setDriveVelocity(double velocity) {
		driveClosedLoop = true;
		driveSetpoint_mps = velocity;
	}

	@Override
	public void setAngleVoltage(double voltage) {
		angleClosedLoop = false;
		angleController.reset();
		angleApplied_V = MathUtil.clamp(voltage, -12.0, 12.0);
		angleSim.setInputVoltage(angleApplied_V);
	}

	@Override
	public void setAnglePosition(Rotation2d angle) {
		angleClosedLoop = true;
		angleSetpoint_rad = angle.getRadians();
	}

	@Override
	public double getDriveCharacterizationVelocity_radps() {
		return driveSim.getAngularVelocityRadPerSec();
	}
}
