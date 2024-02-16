package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;
import static frc.robot.constants.Constants.tuningMode;

import frc.robot.util.AutoSetterTunableNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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
		leadMotor.setSmartCurrentLimit(45);
		leadMotor.setSecondaryCurrentLimit(65);

		followerMotor.setIdleMode(IdleMode.kBrake);
		followerMotor.setSmartCurrentLimit(45);
		followerMotor.setSecondaryCurrentLimit(65);
		followerMotor.follow(leadMotor);

		pidController.setFeedbackDevice(shaftEncoder);
		pidController.setP(kP);
		pidController.setD(kD);

		pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		pidController.setSmartMotionMaxAccel(maxAccel_rpmps, 0);
		pidController.setSmartMotionMaxVelocity(maxVelocity_rpm, 0);
		pidController.setOutputRange(minOutput, maxOutput);

		// revolutions * deg / rev = deg
		shaftEncoder.setPositionConversionFactor(360);

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
		public double leadVoltage_V = 0.0;
		public double leadTemperature_C = 0.0;
		public double leadPercentOutput = 0.0;

		public double followerCurrent_A = 0.0;
		public double followerVoltage_V = 0.0;
		public double followerTemperature_C = 0.0;
		public double followerPercentOutput = 0.0;

		public double shaftPosition_deg = 0.0;
		public double shaftVelocity_rpm = 0.0;
	}

	public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private void updateInputs(ArmIOInputs inputs) {
		inputs.leadCurrent_A = leadMotor.getOutputCurrent();
		inputs.leadVoltage_V = leadMotor.getBusVoltage();
		inputs.leadTemperature_C = leadMotor.getMotorTemperature();
		inputs.leadPercentOutput = leadMotor.getAppliedOutput();

		inputs.followerCurrent_A = followerMotor.getOutputCurrent();
		inputs.followerVoltage_V = followerMotor.getBusVoltage();
		inputs.followerTemperature_C = followerMotor.getMotorTemperature();
		inputs.followerPercentOutput = followerMotor.getAppliedOutput();

		inputs.shaftPosition_deg = shaftEncoder.getPosition() - shaftEncoderOffset_deg;
		inputs.shaftVelocity_rpm = shaftEncoder.getVelocity();
	}

	public void setPosition(double position_deg) {
		targetPosition_deg = position_deg;
		pidController.setReference(targetPosition_deg + shaftEncoderOffset_deg, ControlType.kSmartMotion, 0,
			feedforwardController.calculate(targetPosition_deg, 0));
	}

	public void stopArm() {
		pidController.setReference(0, ControlType.kDutyCycle, 0,
			feedforwardController.calculate(inputs.shaftPosition_deg, 0));
		// Set
	}

	public boolean closeEnough() {
		return MathUtil.isNear(targetPosition_deg, inputs.shaftPosition_deg, accuracyTolerance);
	}
}
