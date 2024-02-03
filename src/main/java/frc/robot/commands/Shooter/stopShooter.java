package frc.robot.commands.Shooter;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class stopShooter extends Command {
	private Shooter shooterM;
	private double targetVelocity = 0;

	public stopShooter(Shooter shooter, double velocity_rpm) {
		shooterM = shooter;
		targetVelocity = velocity_rpm;
		addRequirements(shooter);

	}

	@Override
	public void initialize() {
		shooterM.setVelocity(0);

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
