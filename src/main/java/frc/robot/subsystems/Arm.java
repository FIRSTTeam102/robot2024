package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import frc.robot.util.Math102;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
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

	// feedforward control to keep the arm up
	private ArmFeedforward feedforwardController = new ArmFeedforward(0, kG, 0, 0);

	private Relay climberRelay = new Relay(0);

	@Getter
	@AutoLogOutput
	private double targetPosition_deg = 0;

	public Arm() {
		leadMotor.restoreFactoryDefaults();
		leadMotor.setIdleMode(IdleMode.kBrake);
		leadMotor.setInverted(false);
		leadMotor.setSmartCurrentLimit(45);
		leadMotor.setSecondaryCurrentLimit(65);
		leadMotor.enableVoltageCompensation(12);

		leadMotor.setSoftLimit(SoftLimitDirection.kForward, (float) (110 + shaftEncoderOffset_deg));
		leadMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (-1.5 + shaftEncoderOffset_deg)); // change to allow us
																																																	// to flipover
		leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
		leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

		followerMotor.restoreFactoryDefaults();
		followerMotor.setIdleMode(IdleMode.kBrake);
		followerMotor.setSmartCurrentLimit(45);
		followerMotor.setSecondaryCurrentLimit(65);
		followerMotor.enableVoltageCompensation(12);
		followerMotor.follow(leadMotor, true);

		// revolutions * deg / rev = deg
		shaftEncoder.setPositionConversionFactor(360);
		// rev / sec * sec / min = RPM
		shaftEncoder.setVelocityConversionFactor(60);
		shaftEncoder.setInverted(true);
		shaftEncoder.setZeroOffset(180);

		pidController.setFeedbackDevice(shaftEncoder);
		// Smart motion applies a velocity and acceleration limiter as it travels to the target position. More info can be
		// found here:
		// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
		pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		pidController.setSmartMotionMaxAccel(maxAccel_rpmps, 0);
		pidController.setSmartMotionMaxVelocity(maxVelocity_rpm, 0);
		pidController.setSmartMotionAllowedClosedLoopError(accuracyTolerance_deg, 0);
		pidController.setOutputRange(minOutput, maxOutput);
		// since we are using smartmotion, the PID numbers are for velocity control, not position.
		pidController.setP(kP, 0);
		pidController.setD(kD, 0);
		// Treats 0 and 360 degrees as the same number, so going from one side of 0 to the other doesnt make it do a 360
		pidController.setPositionPIDWrappingEnabled(true);
		pidController.setPositionPIDWrappingMinInput(0);
		pidController.setPositionPIDWrappingMaxInput(360);

		// high angles PID loop
		pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 1);
		pidController.setSmartMotionMaxAccel(maxAccel_rpmps, 1);
		pidController.setSmartMotionMaxVelocity(maxVelocityHigh_rpm, 1);
		pidController.setSmartMotionAllowedClosedLoopError(accuracyToleranceHigh_deg, 1);

		pidController.setP(kPHigh, 1);
		pidController.setD(kDHigh, 1);

		// if (tuningMode) {
		// new AutoSetterTunableNumber("Arm/kP", kP, (value) -> pidController.setP(value));
		// new AutoSetterTunableNumber("Arm/kD", kD, (value) -> pidController.setD(value));
		// }

		leadMotor.burnFlash();
		followerMotor.burnFlash();
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
		public double motorVelocity_rpm = 0.0;
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
		inputs.followerAppliedVoltage_V = 12 * followerMotor.getAppliedOutput();
		inputs.followerTemperature_C = followerMotor.getMotorTemperature();
		inputs.followerPercentOutput = followerMotor.getAppliedOutput();

		inputs.shaftPosition_deg = Math102.truncate(shaftEncoder.getPosition() - shaftEncoderOffset_deg, 2);
		inputs.shaftVelocity_rpm = Math102.truncate(shaftEncoder.getVelocity(), 2);
		inputs.motorVelocity_rpm = Math102.truncate(motorEncoder.getVelocity(), 2);
	}

	public void setPosition(double position_deg, int pidSlot) {
		targetPosition_deg = position_deg;
		pidController.setReference(targetPosition_deg + shaftEncoderOffset_deg, ControlType.kSmartMotion, pidSlot,
			feedforwardController.calculate(Units.degreesToRadians(targetPosition_deg), 0));
	}

	public void setPosition(double position_deg) {
		setPosition(position_deg, 0);
	}

	public void setMotorVoltage(double voltage_V) {
		Logger.recordOutput("Arm/targetVoltage_V", voltage_V);
		pidController.setReference(voltage_V, ControlType.kVoltage, 0,
			feedforwardController.calculate(Units.degreesToRadians(inputs.shaftPosition_deg), 0));
	}

	public void setClimberRelay(Relay.Value value) {
		climberRelay.set(value);
		Logger.recordOutput("Arm/relaySetting", value);
	}

	public void stop() {
		pidController.setReference(0, ControlType.kDutyCycle, 0,
			feedforwardController.calculate(Units.degreesToRadians(inputs.shaftPosition_deg), 0));
		// Set
	}

	public boolean closeEnough() {
		return MathUtil.isNear(targetPosition_deg, inputs.shaftPosition_deg, .8);
	}

	/**
	 * Gets the best PID configuration to use based on target position, actual position, and current driverstation mode
	 * @param targetPosition_deg
	 * @return recommended PID slot, either 0 (normal) or 1 (high angle). Can be fed directly into {@link Arm#setPosition(double, int) setPosition}
	 */
	public int getBestPIDSlot(double targetPosition_deg) {
		// if in autonomous, let it shake
		if (!DriverStation.isAutonomous())
			return 0;

		// in auto, use the dual PID loop
		boolean chooseTop = (inputs.shaftPosition_deg > 65) && (targetPosition_deg > 65);
		return chooseTop ? 1 : 0;
	}

	/**
	 * ~30s climb startup routine to get it in the right position (no position control :(  )
	 * @return Command to be scheduled
	 */
	public Command climbStartup() {
		return Commands.runOnce(() -> setClimberRelay(Relay.Value.kReverse)).andThen(Commands.waitSeconds(15))
			.andThen(() -> setClimberRelay(Relay.Value.kForward)).andThen(Commands.waitSeconds(0))
			.andThen(() -> setClimberRelay(Relay.Value.kOn));
	}
}
