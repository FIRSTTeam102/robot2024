package frc.robot.subsystems;

import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);

	private final DigitalInput noteSensor = new DigitalInput(sensorId);

	public Intake() {
		motor.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);
	}

	public void setMotorSpeed(double speed) {
		motor.set(speed);
	}

	public void setMotorVoltage(double voltage_V) {
		motor.setVoltage(voltage_V);
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

	public boolean detectNote() {
		return inputs.noteSensor;

	}

	public void stopMotor() {
		motor.stopMotor();
	}

}