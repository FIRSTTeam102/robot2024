package frc.robot.subsystems;

import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Intake extends SubsystemBase {
	private final CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);

	private final DigitalInput noteSensor = new DigitalInput(sensorId);

	public Intake() {}

	@Override
	public void periodic() {
	}

	public boolean checkSensor() {
		return noteSensor.get();
	}

	public void setMotorSpeed(double speed) {
		motor.set(speed);
		return;
	}

	public void 
}
