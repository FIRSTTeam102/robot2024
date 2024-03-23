package frc.robot.commands.intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LightsConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
		if (!intake.isHoldingNote()) {
			arm.setPosition(-2);
			intake.setMotorSpeed(IntakeConstants.intakeSpeed);
			Lights.setStatus(LightsConstants.Mode.Intaking);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (DriverStation.isAutonomous())
			arm.setPosition(4);
		else
			arm.setPosition(40);
		Commands.waitSeconds(.015).andThen(intake::stopMotor, intake).schedule();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return intake.isHoldingNote();
	}
}
