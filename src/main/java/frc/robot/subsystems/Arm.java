package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);
	private SparkPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();

	private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

	@Getter
	@AutoLogOutput
	private double targetPosition_rad = 0;

	public Arm() {
		motor.setIdleMode(IdleMode.kBrake);

		pidController.setP(kP);
		pidController.setD(kD);
		pidController.setI(kI);

		motor.setSmartCurrentLimit(50);
		motor.setSecondaryCurrentLimit(65);

		encoder.setPositionConversionFactor((1 / gearRatio) * 2 * Math.PI); // motor revolutions * arm rev / motor rev * arm
		// radians / arm rev = arm radians
		encoder.setVelocityConversionFactor((1 / gearRatio) * (2 * Math.PI) * (1 / 60)); // RPM * arm rev / motor rev *
																																											// radians /
		// revolution * min / sec = rad/s
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);
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

	public void setPosition(double position_rad) {
		targetPosition_rad = position_rad;
		pidController.setReference(targetPosition_rad, ControlType.kPosition, 0,
			feedforwardController.calculate(targetPosition_rad, 0));
	}

	public void stopArm() {
		pidController.setReference(0, ControlType.kDutyCycle, 0, feedforwardController.calculate(inputs.position_rad, 0));
		// Set
	}

	public boolean closeEnough() {
		if (MathUtil.isNear(targetPosition_rad, inputs.position_rad, closeVar)) {
			return true;
		} else {
			return false;
		}
	}
}
