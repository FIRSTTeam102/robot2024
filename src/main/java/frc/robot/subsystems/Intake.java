package frc.robot.subsystems;

import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Intake extends SubsystemBase {
	private final CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);

	private final DigitalInput noteSensor = new DigitalInput(sensorId);

	@Getter
	@AutoLogOutput
	private boolean holdingNote = false;
	private boolean cachedNoteSensor = false;

	public Intake() {
		motor.restoreFactoryDefaults();
		motor.setIdleMode(IdleMode.kBrake);
		motor.enableVoltageCompensation(12);
		motor.burnFlash();

		Shuffleboard.getTab("drive").addBoolean("Holding Note?", () -> holdingNote).withWidget(BuiltInWidgets.kBooleanBox);

	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);

		if ((inputs.noteSensor != cachedNoteSensor) && inputs.noteSensor) {
			holdingNote = !holdingNote;
		}

		cachedNoteSensor = inputs.noteSensor;
	}

	public void setMotorSpeed(double speed) {
		speed = MathUtil.clamp(speed, -1, 1);
		motor.setVoltage(speed * 12);
	}

	/**
	 * Manually set the note sensor and corresponding holding logic to a specific value
	 * @param value
	 */
	public void resetNoteDetection(boolean value) {
		holdingNote = value;
		cachedNoteSensor = value;
	}

	/**
	 * Default version of {@link Intake#resetNoteDetection(boolean)} with false as the default value.
	 */
	public void resetNoteDetection() {
		resetNoteDetection(false);
	}

	@AutoLog
	public static class IntakeIOInputs {
		public double current_A = 0.0;
		public double voltage_V = 0.0;
		public double tempature_C = 0.0;
		public double percentOutput = 0.0;
		public boolean noteSensor = false;
	}

	public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	public void updateInputs(IntakeIOInputs inputs) {
		inputs.current_A = motor.getOutputCurrent();
		inputs.voltage_V = motor.getBusVoltage();
		inputs.tempature_C = motor.getMotorTemperature();
		inputs.percentOutput = motor.getAppliedOutput();
		inputs.noteSensor = !noteSensor.get();
	}

	public void stopMotor() {
		motor.stopMotor();
	}

}