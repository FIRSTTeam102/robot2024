package frc.robot.commands.arm;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class ManualArmControl extends Command {
	public DoubleSupplier speedSupplier;
	public Arm arm;

	/**
	 * Constructs a ManualArmControl command, using the provided function to supply speed numbers
	 * @param arm Arm subsystem to be passed in
	 * @param speedSupplier function that supplies the speed used to run the arm
	 */
	public ManualArmControl(Arm arm, DoubleSupplier speedSupplier) {
		this.arm = arm;
		this.speedSupplier = speedSupplier;
		addRequirements(arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// scale speed linearly by max output and then scale by 12 V for 100%
		arm.setMotorVoltage((ArmConstants.maxOutput * speedSupplier.getAsDouble()) * 12);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		arm.stopArm();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
