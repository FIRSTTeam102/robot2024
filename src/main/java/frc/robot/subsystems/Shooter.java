package frc.robot.subsystems;

import static frc.robot.constants.Constants.tuningMode;
import static frc.robot.constants.ShooterConstants.*;

import frc.robot.util.AutoSetterTunableNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, CANSparkMax.MotorType.kBrushless);
	private SparkPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

	public Shooter() {
		if (tuningMode) {
			new AutoSetterTunableNumber("Shooter/kP", kP, (double value) -> pidController.setP(value));
			new AutoSetterTunableNumber("Shooter/kD", kD, (double value) -> pidController.setD(value));

			new AutoSetterTunableNumber("Shooter/setpoint", 0, (double value) -> setVelocity(value));
		}

		pidController.setP(kP);
		pidController.setI(0);
		pidController.setD(kD);
	}

	public void setVelocity(double velocity_rpm) {
		pidController.setReference(velocity_rpm, ControlType.kVelocity, 0, feedforward.calculate(velocity_rpm));
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);
	}

	@AutoLog
	public static class ShooterIOInputs {
		public double position_rot = 0.0;
		public double velocity_rpm = 0.0;
		public double voltage_V = 0.0;
		public double current_A = 0.0;
		public double temperature_C = 0.0;
	}

	public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

	private void updateInputs(ShooterIOInputs inputs) {
		inputs.position_rot = encoder.getPosition();
		inputs.velocity_rpm = encoder.getVelocity();
		inputs.voltage_V = motor.getBusVoltage() * motor.getAppliedOutput();
		inputs.current_A = motor.getOutputCurrent();
		inputs.temperature_C = motor.getMotorTemperature();
	}

	public final SysIdRoutine sysIdRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(
			(voltage) -> {
				pidController.setReference(voltage.in(Units.Volts), ControlType.kVoltage);
			},
			log -> {
				log.motor("shooter")
					.voltage(Units.Volts.of(inputs.voltage_V))
					.angularPosition(Units.Rotations.of(inputs.position_rot))
					.angularVelocity(Units.RPM.of(inputs.velocity_rpm));
			},
			this));
}
