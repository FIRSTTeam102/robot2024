package frc.robot.swerve;

import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.Conversions;
import frc.robot.util.TunableNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;

/**
 * Implementation of the SwerveModuleIO interface for MK4i Swerve Modules with
 * a NEO (SparkMax) for drive, NEO (SparkMax) for turn, and a Thrifty encoder
 */
public class SwerveModuleIOReal implements SwerveModuleIO {
	private CANSparkMax driveMotor;
	private SparkPIDController driveSparkPidController;
	private RelativeEncoder driveRelativeEncoder;
	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

	private CANSparkMax angleMotor;
	private SparkPIDController angleSparkPidController;
	private RelativeEncoder angleRelativeEncoder;
	private SparkAnalogSensor angleAbsoluteEncoder;
	private double angleOffset_rad;
	// private SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(angleKs, angleKv, angleKa);

	private TunableNumber tunableDriveKp;
	private TunableNumber tunableDriveKd;
	private TunableNumber tunableAngleKp;
	private TunableNumber tunableAngleKd;

	/**
	 * @param moduleConstants module config
	 * @param moduleNumber module number used for identification
	 */
	public SwerveModuleIOReal(SwerveModuleConstants moduleConstants, int moduleNumber) {
		this.angleOffset_rad = moduleConstants.angleOffset_rad();

		final String key = "SwerveModuleIO" + moduleNumber + "/";
		tunableDriveKp = new TunableNumber(key + "driveKp", driveKp);
		tunableDriveKd = new TunableNumber(key + "driveKd", driveKd);
		tunableAngleKp = new TunableNumber(key + "angleKp", angleKp);
		tunableAngleKd = new TunableNumber(key + "angleKd", angleKd);

		driveMotor = new CANSparkMax(moduleConstants.driveMotorId(), CANSparkMax.MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveSparkPidController = driveMotor.getPIDController();
		driveSparkPidController.setP(driveKp);
		driveSparkPidController.setI(driveKi);
		driveSparkPidController.setD(driveKd);
		driveSparkPidController.setFF(driveKf);
		// driveSparkPidController.setOutputRange(-driveMaxPercentOutput, driveMaxPercentOutput);
		driveMotor.setSmartCurrentLimit(driveCurrentLimit_amp);
		driveMotor.setInverted(driveInverted);
		driveMotor.setIdleMode(driveIdleMode);
		driveMotor.setClosedLoopRampRate(driveRampTime_s);
		driveMotor.enableVoltageCompensation(12);

		driveRelativeEncoder = driveMotor.getEncoder();
		driveRelativeEncoder.setPositionConversionFactor(wheelCircumference_m / driveGearRatio); // m
		driveRelativeEncoder.setVelocityConversionFactor(wheelCircumference_m / driveGearRatio / 60.0); // mps

		// turn down frequency as we only log them, not needed for calculations
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40); // percent output
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40); // velocity, current
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40); // position
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); // alternate encoder
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); // duty cycle position
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // duty cycle velocity

		driveMotor.burnFlash();

		angleMotor = new CANSparkMax(moduleConstants.angleMotorId(), CANSparkMax.MotorType.kBrushless);
		angleMotor.restoreFactoryDefaults();
		angleSparkPidController = angleMotor.getPIDController();
		angleSparkPidController.setP(angleKp);
		angleSparkPidController.setI(angleKi);
		angleSparkPidController.setD(angleKd);
		angleSparkPidController.setFF(angleKf);
		// angleSparkPidController.setOutputRange(-angleMaxPercentOutput, angleMaxPercentOutput);
		angleMotor.setSmartCurrentLimit(angleCurrentLimit_amp);
		angleMotor.setInverted(angleInverted);
		angleMotor.setIdleMode(angleIdleMode);
		// angleMotor.setClosedLoopRampRate(angleRampTime_s);
		angleMotor.enableVoltageCompensation(12);

		angleRelativeEncoder = angleMotor.getEncoder();
		angleRelativeEncoder.setPositionConversionFactor(Conversions.twoPi / angleGearRatio); // rad
		angleRelativeEncoder.setVelocityConversionFactor(Conversions.twoPi / angleGearRatio / 60.0); // radps

		driveRelativeEncoder.setPositionConversionFactor(wheelCircumference_m / driveGearRatio); // m
		driveRelativeEncoder.setVelocityConversionFactor(wheelCircumference_m / driveGearRatio / 60.0); // mps

		angleAbsoluteEncoder = angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
		angleAbsoluteEncoder.setPositionConversionFactor(Conversions.twoPi / 3.3 /* V */); // rad
		// absolute encoder velocity reading is meaningless (jumps around), using relative encoder's instead
		// angleAbsoluteEncoder.setVelocityConversionFactor(Conversions.twoPi / 3.3 /* V */); // radps
		angleSparkPidController.setFeedbackDevice(angleAbsoluteEncoder);

		// wrap betwee 0 and 2pi radians
		angleSparkPidController.setPositionPIDWrappingEnabled(true);
		angleSparkPidController.setPositionPIDWrappingMinInput(0);
		angleSparkPidController.setPositionPIDWrappingMaxInput(Conversions.twoPi);

		// turn down frequency as we only log them, not needed for calculations
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40); // percent output
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40); // velocity, current
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // position
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); // alternate encoder
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); // duty cycle position
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // duty cycle velocity

		angleMotor.burnFlash();
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		inputs.driveDistance_m = driveRelativeEncoder.getPosition();
		inputs.driveVelocity_mps = driveRelativeEncoder.getVelocity();
		inputs.driveVoltage_V = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
		inputs.driveCurrent_A = driveMotor.getOutputCurrent();
		inputs.driveTemperature_C = driveMotor.getMotorTemperature();

		inputs.anglePosition_rad = angleRelativeEncoder.getPosition();
		inputs.angleAbsolutePosition_rad = Conversions
			.angleModulus2pi(angleAbsoluteEncoder.getPosition() - angleOffset_rad); // ensure offset is properly wrapped
		// inputs.angleVelocity_radps = angleAbsoluteEncoder.getVelocity(); // gives garbage data
		inputs.angleVelocity_radps = angleRelativeEncoder.getVelocity();
		inputs.angleVoltage_V = angleMotor.getBusVoltage() * angleMotor.getAppliedOutput();
		inputs.angleCurrent_A = angleMotor.getOutputCurrent();
		inputs.angleTemperature_C = angleMotor.getMotorTemperature();
	}

	@Override
	public void setDriveVoltage(double voltage) {
		driveMotor.setVoltage(voltage);
	}

	@Override
	public void setDriveVelocity(double velocity) {
		driveSparkPidController.setReference(velocity, ControlType.kVelocity, 0, driveFeedforward.calculate(velocity));
	}

	@Override
	public void setAngleVoltage(double voltage) {
		angleMotor.setVoltage(voltage);
	}

	@Override
	public void setAnglePosition(Rotation2d angle) {
		final double targetAngle_rad = Conversions.angleModulus2pi(angle.getRadians());
		angleSparkPidController.setReference(targetAngle_rad, ControlType.kPosition);
		// 0, angleFeedforward.calculate(targetAngle_rad));
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setOffset(double offset_rad) {
		angleOffset_rad = offset_rad;
	}

	@Override
	public double getDriveCharacterizationVelocity_radps() {
		return Conversions.twoPi * (driveRelativeEncoder.getVelocity() / wheelCircumference_m); // undo unit conversion
	}

	@Override
	public void tunablePeriodic() {
		if (tunableDriveKp.hasChanged())
			driveSparkPidController.setP(tunableDriveKp.get());
		if (tunableDriveKd.hasChanged())
			driveSparkPidController.setD(tunableDriveKd.get());
		if (tunableAngleKp.hasChanged())
			angleSparkPidController.setP(tunableAngleKp.get());
		if (tunableAngleKd.hasChanged())
			angleSparkPidController.setD(tunableAngleKd.get());
	}
}
