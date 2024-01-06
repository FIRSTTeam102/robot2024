package frc.robot.constants;

public final class Constants {
	public static final double loopPeriod_s = org.littletonrobotics.junction.LoggedRobot.defaultPeriodSecs; // edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod

	public static final RobotMode robotMode = RobotMode.Active;

	public enum RobotMode {
		Active, Replay;
	}

	public static class OperatorConstants {
		public static final int driverControllerPort = 0;

		public static final double xboxStickDeadband = 0.1;
		public static final double boolTriggerThreshold = 0.3;
	}

	public static final int pdhId = 1;
	public static final int pigeonId = 3;

	public static class ShuffleboardConstants {
		public static final String driveTab = "drive";
	}
}
