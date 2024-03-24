package frc.robot.commands.intake;

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

	private IntakeWithArm(Intake intake, Arm arm) {
		this.intake = intake;
		this.arm = arm;
		addRequirements(intake, arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (!intake.isHoldingNote()) {
			arm.setPosition(-2);
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
		// Commands.waitSeconds(.015).andThen(intake::stopMotor, intake).schedule();
		if (interrupted) {
			intake.stopMotor();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return intake.isHoldingNote();
	}

	public static Command intakeWithDelay(Intake intake, Arm arm) {
		return new IntakeWithArm(intake, arm).andThen(Commands.waitSeconds(.015)).andThen(intake::stopMotor, intake);
	}
}
