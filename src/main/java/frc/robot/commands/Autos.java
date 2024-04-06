package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * @deprecated All autonomous stuff done with Choreo, PathPlanner, and inlines
 */
public final class Autos {

	public static Command deadlineSeconds(double time_s, Command... commands) {
		return Commands.deadline(Commands.waitSeconds(time_s), commands);

	}
}
