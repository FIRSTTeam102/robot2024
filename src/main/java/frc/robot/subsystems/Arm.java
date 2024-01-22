package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);
	private SparkPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();

	private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

	@Getter
	private double targetPosition_rad = 0;

	public Arm() {
		motor.setIdleMode(IdleMode.kBrake);

		pidController.setP(kP);
		pidController.setD(kD);
		pidController.setI(kI);

		motor.setSmartCurrentLimit(50);
		motor.setSecondaryCurrentLimit(65);

		encoder.setPositionConversionFactor(2 * Math.PI); // revolutions * radians / revolution = radians
		encoder.setVelocityConversionFactor(2 * Math.PI * (1 / 60)); // RPM * radians / revolution * min / sec = rad/s
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);

		Logger.recordOutput("Elevator/targetPosition_rad", targetPosition_rad);
	}

	@AutoLog
	public static class ArmIOInputs {
		public double current_A = 0.0;
		public double voltage_V = 0.0;
		public double temperature_C = 0.0;
		public double position_rad = 0.0;
		public double velocity_radps = 0.0;
	}

	public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private void updateInputs(ArmIOInputs inputs) {
		inputs.current_A = motor.getOutputCurrent();
		inputs.voltage_V = motor.getBusVoltage();
		inputs.temperature_C = motor.getMotorTemperature();
		inputs.position_rad = encoder.getPosition();
		inputs.velocity_radps = encoder.getVelocity();
	}
}
