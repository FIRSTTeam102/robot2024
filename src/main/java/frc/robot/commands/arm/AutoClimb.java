package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoClimb extends Command {
	private Arm arm;

	public AutoClimb(Arm arm) {
		addRequirements(arm);
		this.arm = arm;
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		boolean nearZero = MathUtil.isNear(0, arm.inputs.shaftPosition_deg, 1);
		if (!nearZero)
			arm.setPosition(0);
		else
			arm.setMotorVoltage(-.75);
	}

	@Override
	public void end(boolean interrupted) {
		arm.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
