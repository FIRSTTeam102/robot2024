package frc.robot.subsystems;

import static frc.robot.constants.Constants.tuningMode;
import static frc.robot.constants.ShooterConstants.*;

import frc.robot.util.AutoSetterTunableNumber;
import frc.robot.util.SparkUtil;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Shooter extends SubsystemBase {
	private CANSparkMax leadMotor = new CANSparkMax(leadMotorId, CANSparkMax.MotorType.kBrushless);
	private CANSparkMax followerMotor = new CANSparkMax(followerMotorId, CANSparkMax.MotorType.kBrushless);
	private SparkPIDController pidController = leadMotor.getPIDController();
	private RelativeEncoder encoder = leadMotor.getEncoder();
	private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

	@Getter
	@AutoLogOutput
	private double targetVelocity_rpm = 0.0;

	public Shooter() {
		if (tuningMode) {
			new AutoSetterTunableNumber("Shooter/kP", kP, (double value) -> pidController.setP(value));
			new AutoSetterTunableNumber("Shooter/kD", kD, (double value) -> pidController.setD(value));

			new AutoSetterTunableNumber("Shooter/setpoint", 0, (double value) -> setVelocity(value));
		}

		pidController.setP(kP);
		pidController.setI(0);
		pidController.setD(kD);

		leadMotor.setIdleMode(IdleMode.kCoast);

		followerMotor.setIdleMode(IdleMode.kCoast);
		followerMotor.follow(leadMotor);

		// only fetch position when tuning
		SparkUtil.setPeriodicFrames(leadMotor, true, true, tuningMode, false, false, false, false);
		SparkUtil.setPeriodicFrames(followerMotor, false, false, false, false, false, false, false);
	}

	public void setVelocity(double velocity_rpm) {
		targetVelocity_rpm = velocity_rpm;
		pidController.setReference(velocity_rpm, ControlType.kVelocity, 0, feedforward.calculate(velocity_rpm));
	}

	public void setPercentOutput(double speed) {
		pidController.setReference(speed, ControlType.kDutyCycle);
	}

	public void stop() {
		pidController.setReference(0, ControlType.kDutyCycle);
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);
	}

	@AutoLog
	public static class ShooterIOInputs {
		// public double leadPosition_rot = 0.0;
		public double leadVelocity_rpm = 0.0;
		public double leadVoltage_V = 0.0;
		public double leadCurrent_A = 0.0;
		public double leadTemperature_C = 0.0;
		public double leadPercentOutput = 0.0;

		public double followerVoltage_V = 0.0;
		public double followerCurrent_A = 0.0;
		public double followerTemperature_C = 0.0;
		public double followerPercentOutput = 0.0;
	}

	public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	private void updateInputs(ShooterIOInputs inputs) {
		// inputs.leadPosition_rot = encoder.getPosition();
		inputs.leadVelocity_rpm = encoder.getVelocity();
		inputs.leadVoltage_V = leadMotor.getBusVoltage();
		inputs.leadCurrent_A = leadMotor.getOutputCurrent();
		inputs.leadTemperature_C = leadMotor.getMotorTemperature();
		inputs.leadPercentOutput = leadMotor.getAppliedOutput();

		// important to log to ensure proper functionality
		inputs.followerVoltage_V = followerMotor.getBusVoltage();
		inputs.followerCurrent_A = followerMotor.getOutputCurrent();
		inputs.followerTemperature_C = followerMotor.getMotorTemperature();
		inputs.followerPercentOutput = followerMotor.getAppliedOutput();
	}

	public final SysIdRoutine sysIdRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(
			(voltage) -> {
				pidController.setReference(voltage.in(Units.Volts), ControlType.kVoltage);
			},
			log -> {
				log.motor("shooter-lead")
					.voltage(Units.Volts.of(inputs.leadVoltage_V))
					.angularPosition(Units.Rotations.of(encoder.getPosition()))
					.angularVelocity(Units.RPM.of(inputs.leadVelocity_rpm));
			},
			this));
}
