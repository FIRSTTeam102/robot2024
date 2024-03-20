package frc.robot.commands.scoring;

import frc.robot.constants.ScoringConstants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class SetScoringPosition extends Command {
	private Arm arm;
	private Shooter shooter;

	private Supplier<ScoringPosition> posSupplier;

	/**
	 * Create a set scoring position command that adjusts dynamically according to a method that provides a {@link ScoringPosition} object 
	 * @param arm Arm subsystem
	 * @param shooter Shooter subsystem
	 * @param posSupplier Method that provides a {@link ScoringPosition}. This method is called on initialize
	 */
	public SetScoringPosition(Arm arm, Shooter shooter, Supplier<ScoringPosition> posSupplier) {
		addRequirements(arm, shooter);
		this.arm = arm;
		this.shooter = shooter;
		this.posSupplier = posSupplier;
	}

	/**
	 * Create a set scoring position command that goes to a static {@link ScoringPosition}
	 * @param arm Arm subsystem
	 * @param shooter Shooter subsyste,
	 * @param position ScoringPosition to go to
	 */
	public SetScoringPosition(Arm arm, Shooter shooter, ScoringPosition position) {
		this(arm, shooter, () -> position);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		var targetPosition = posSupplier.get();
		arm.setPosition(targetPosition.armAngle_deg());
		shooter.setVelocity(targetPosition.shooterSpeed_rpm());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			shooter.stop();
			arm.stop();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return arm.closeEnough();
	}
}
