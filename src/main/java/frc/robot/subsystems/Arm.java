package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;
import static frc.robot.constants.Constants.tuningMode;

import frc.robot.util.AutoSetterTunableNumber;
import frc.robot.util.Math102;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Arm extends SubsystemBase {
	private CANSparkMax leadMotor = new CANSparkMax(leadMotorId, MotorType.kBrushless);
	private CANSparkMax followerMotor = new CANSparkMax(followerMotorId, MotorType.kBrushless);
	private SparkPIDController pidController = leadMotor.getPIDController();
	private RelativeEncoder motorEncoder = leadMotor.getEncoder(); // built-in encoder in the lead NEO
	// throughbore encoder on hex shaft
	private AbsoluteEncoder shaftEncoder = leadMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

	private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

	@Getter
	@AutoLogOutput
	private double targetPosition_deg = 0;

	public Arm() {
		leadMotor.setIdleMode(IdleMode.kBrake);
		leadMotor.setInverted(true);
		leadMotor.setSmartCurrentLimit(45);
		leadMotor.setSecondaryCurrentLimit(65);
		leadMotor.enableVoltageCompensation(12);

		followerMotor.setIdleMode(IdleMode.kBrake);
		followerMotor.setSmartCurrentLimit(45);
		followerMotor.setSecondaryCurrentLimit(65);
		followerMotor.enableVoltageCompensation(12);
		followerMotor.follow(leadMotor, true);

		// revolutions * deg / rev = deg
		shaftEncoder.setPositionConversionFactor(360);
		shaftEncoder.setInverted(true);
		shaftEncoder.setZeroOffset(shaftEncoderOffset_deg);

		pidController.setFeedbackDevice(shaftEncoder);
		// Smart motion applies a velocity and acceleration limiter as it travels to the target position. More info can be
		// found here:
		// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
		pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		pidController.setSmartMotionMaxAccel(maxAccel_rpmps, 0);
		pidController.setSmartMotionMaxVelocity(maxVelocity_rpm, 0);
		pidController.setOutputRange(minOutput, maxOutput);
		// since we are using smartmotion, the PID numbers are for velocity control, not position.
		pidController.setP(kP);
		pidController.setD(kD);
		// Treats 0 and 360 degrees as the same number, so going from one side of 0 to the other doesnt make it do a 360
		pidController.setPositionPIDWrappingEnabled(true);
		pidController.setPositionPIDWrappingMinInput(0);
		pidController.setPositionPIDWrappingMaxInput(360);

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
		public double leadCurrent_A = 0.0;
		public double leadBusVoltage_V = 0.0;
		public double leadAppliedVoltage_V = 0.0;
		public double leadTemperature_C = 0.0;
		public double leadPercentOutput = 0.0;

		public double followerCurrent_A = 0.0;
		public double followerBusVoltage_V = 0.0;
		public double followerAppliedVoltage_V = 0.0;
		public double followerTemperature_C = 0.0;
		public double followerPercentOutput = 0.0;

		public double shaftPosition_deg = 0.0;
		public double shaftVelocity_rpm = 0.0;
	}

	public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private void updateInputs(ArmIOInputs inputs) {
		inputs.leadCurrent_A = leadMotor.getOutputCurrent();
		inputs.leadBusVoltage_V = leadMotor.getBusVoltage();
		inputs.leadAppliedVoltage_V = 12 * leadMotor.getAppliedOutput();
		inputs.leadTemperature_C = leadMotor.getMotorTemperature();
		inputs.leadPercentOutput = leadMotor.getAppliedOutput();

		inputs.followerCurrent_A = followerMotor.getOutputCurrent();
		inputs.followerBusVoltage_V = followerMotor.getBusVoltage();
		inputs.leadAppliedVoltage_V = 12 * followerMotor.getAppliedOutput();
		inputs.followerTemperature_C = followerMotor.getMotorTemperature();
		inputs.followerPercentOutput = followerMotor.getAppliedOutput();

		inputs.shaftPosition_deg = Math102.truncate(shaftEncoder.getPosition(), 2);
		inputs.shaftVelocity_rpm = Math102.truncate(shaftEncoder.getVelocity(), 2);
	}

	public void setPosition(double position_deg) {
		targetPosition_deg = position_deg;
		pidController.setReference(targetPosition_deg, ControlType.kSmartMotion, 0,
			feedforwardController.calculate(Units.degreesToRadians(targetPosition_deg), 0));
	}

	public void setMotorVoltage(double voltage_V) {
		pidController.setReference(voltage_V, ControlType.kVoltage, 0,
			feedforwardController.calculate(Units.degreesToRadians(inputs.shaftPosition_deg), 0));
	}

	public void stop() {
		pidController.setReference(0, ControlType.kDutyCycle, 0,
			feedforwardController.calculate(Units.degreesToRadians(inputs.shaftPosition_deg), 0));
		// Set
	}

	public boolean closeEnough() {
		return MathUtil.isNear(targetPosition_deg, inputs.shaftPosition_deg, accuracyTolerance_deg);
	}
}
