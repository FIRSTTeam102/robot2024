package frc.robot.subsystems;

import static frc.robot.constants.ShooterConstants.*;

import frc.robot.constants.Constants;
import frc.robot.util.AutoSetterTunableNumber;
import frc.robot.util.SparkUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

	@Getter
	@AutoLogOutput
	private boolean atTargetVelocity = false;

	public Shooter() {
		leadMotor.restoreFactoryDefaults();
		followerMotor.restoreFactoryDefaults();
		if (Constants.tuningMode) {
			new AutoSetterTunableNumber("Shooter/kP", kP, (double value) -> pidController.setP(value));
			new AutoSetterTunableNumber("Shooter/kD", kD, (double value) -> pidController.setD(value));

			new AutoSetterTunableNumber("Shooter/setpoint", 0, (double value) -> setVelocity(value));
		}

		// so drivers don't shoot too early
		Shuffleboard.getTab("drive")
			.addBoolean("Shooter at Target Speed?", this::isAtTargetVelocity)
			.withWidget(BuiltInWidgets.kBooleanBox);

		pidController.setP(kP);
		pidController.setI(0);
		pidController.setD(kD);

		leadMotor.setIdleMode(IdleMode.kCoast);
		leadMotor.enableVoltageCompensation(12);

		followerMotor.setIdleMode(IdleMode.kCoast);
		followerMotor.enableVoltageCompensation(12);
		followerMotor.follow(leadMotor);

		// only fetch position when tuning
		SparkUtil.setPeriodicFrames(leadMotor, true, true, Constants.tuningMode, false, false, false, false);
		SparkUtil.setPeriodicFrames(followerMotor, false, false, false, false, false, false, false);
		leadMotor.burnFlash();
		followerMotor.burnFlash();
	}

	public void setVelocity(double velocity_rpm) {
		targetVelocity_rpm = MathUtil.clamp(velocity_rpm, 0, maxVelocity_rpm);
		pidController.setReference(targetVelocity_rpm, ControlType.kVelocity, 0, feedforward.calculate(targetVelocity_rpm));
	}

	public void setPercentOutput(double speed) {
		pidController.setReference(speed * 12, ControlType.kVoltage, 0, 0);
	}

	public void stop() {
		pidController.setReference(0, ControlType.kVoltage);
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);

		atTargetVelocity = MathUtil.isNear(targetVelocity_rpm, inputs.leadVelocity_rpm, 50);
	}

	@AutoLog
	public static class ShooterIOInputs {
		// public double leadPosition_rot = 0.0;
		public double leadVelocity_rpm = 0.0;
		public double leadBusVoltage_V = 0.0;
		public double leadAppliedVoltage_V = 0.0;
		public double leadCurrent_A = 0.0;
		public double leadTemperature_C = 0.0;
		public double leadPercentOutput = 0.0;

		public double followerBusVoltage_V = 0.0;
		public double followerAppliedVoltage_V = 0.0;
		public double followerCurrent_A = 0.0;
		public double followerTemperature_C = 0.0;
		public double followerPercentOutput = 0.0;
	}

	public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	private void updateInputs(ShooterIOInputs inputs) {
		// inputs.leadPosition_rot = encoder.getPosition();
		inputs.leadVelocity_rpm = encoder.getVelocity();
		inputs.leadBusVoltage_V = leadMotor.getBusVoltage();
		inputs.leadAppliedVoltage_V = leadMotor.getAppliedOutput() * 12;
		inputs.leadCurrent_A = leadMotor.getOutputCurrent();
		inputs.leadTemperature_C = leadMotor.getMotorTemperature();
		inputs.leadPercentOutput = leadMotor.getAppliedOutput();

		// important to log to ensure proper functionality
		inputs.followerBusVoltage_V = followerMotor.getBusVoltage();
		inputs.followerAppliedVoltage_V = followerMotor.getAppliedOutput() * 12;
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
					.voltage(Units.Volts.of(inputs.leadAppliedVoltage_V))
					.angularPosition(Units.Rotations.of(encoder.getPosition()))
					.angularVelocity(Units.RPM.of(inputs.leadVelocity_rpm));
			},
			this));
}
