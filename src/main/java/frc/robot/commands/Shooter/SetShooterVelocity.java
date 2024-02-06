package frc.robot.commands.Shooter;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterVelocity extends Command {
	private Shooter shooter;
	private double targetVelocity = 10;

	public SetShooterVelocity(Shooter shooter, double velocity_rpm) {
		this.shooter = shooter;
		targetVelocity = velocity_rpm;
		addRequirements(shooter);

	}

	@Override
	public void initialize() {
		shooter.setVelocity(targetVelocity);

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
