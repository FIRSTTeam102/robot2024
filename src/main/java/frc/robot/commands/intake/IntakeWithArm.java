package frc.robot.commands.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeWithArm extends Command {
	private Intake intake;
	private Arm arm;

	public IntakeWithArm(Intake intake, Arm arm) {
		this.intake = intake;
		this.arm = arm;
		addRequirements(intake, arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (!intake.inputs.noteSensor) {
			arm.setPosition(-2);
			intake.setMotorSpeed(IntakeConstants.intakeSpeed);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		arm.setPosition(10);
		intake.stopMotor();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return intake.inputs.noteSensor;
	}
}
