package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;
import static frc.robot.constants.Constants.tuningMode;

import frc.robot.util.AutoSetterTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

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
	private double targetPosition_deg = 0;

	public Arm() {
		motor.setIdleMode(IdleMode.kBrake);

		pidController.setP(kP);
		pidController.setD(kD);

		pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		pidController.setSmartMotionMaxAccel(maxAccel_rpmps, 0);
		pidController.setSmartMotionMaxVelocity(maxVelocity_rpm, 0);

		motor.setSmartCurrentLimit(45);
		motor.setSecondaryCurrentLimit(65);

		// motor revolutions * arm rev / motor rev * arm degrees / arm rev = arm degrees
		encoder.setPositionConversionFactor((1 / gearRatio) * 360);
		// // RPM * arm rev / motor rev * degrees / revolution * min / sec = deg/s
		// encoder.setVelocityConversionFactor((1 / gearRatio) * 360 * (1 / 60));

		if (tuningMode) {
			new AutoSetterTunableNumber("Arm/kP", kP, (value) -> pidController.setP(value));
			new AutoSetterTunableNumber("Arm/kD", kD, (value) -> pidController.setD(value));
		}
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
		public double position_deg = 0.0;
		public double velocity_rpm = 0.0;
	}

	public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private void updateInputs(ArmIOInputs inputs) {
		inputs.current_A = motor.getOutputCurrent();
		inputs.voltage_V = motor.getBusVoltage();
		inputs.temperature_C = motor.getMotorTemperature();
		inputs.position_deg = encoder.getPosition();
		inputs.velocity_rpm = encoder.getVelocity();
	}

	public void setPosition(double position_deg) {
		targetPosition_deg = position_deg;
		pidController.setReference(targetPosition_deg, ControlType.kSmartMotion, 0,
			feedforwardController.calculate(targetPosition_deg, 0));
	}

	public void stopArm() {
		pidController.setReference(0, ControlType.kDutyCycle, 0, feedforwardController.calculate(inputs.position_deg, 0));
		// Set
	}

	public boolean closeEnough() {
		return MathUtil.isNear(targetPosition_deg, inputs.position_deg, closeVar);
	}
}
