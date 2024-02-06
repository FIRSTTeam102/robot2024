package frc.robot.commands.Shooter;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class StopShooter extends Command {
	private Shooter shooter;

	public StopShooter(Shooter shooter) {
		this.shooter = shooter;
		addRequirements(shooter);

	}

	@Override
	public void initialize() {
		shooter.stop();

	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return true;
	}
}
