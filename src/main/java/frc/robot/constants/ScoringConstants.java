package frc.robot.constants;

import java.util.HashMap;

public final class ScoringConstants {
	/**<p> Represents a configuration for the scoring mechanisms of the robot (arm and shooter)
	 * in their native units (degrees from horizontal and RPM respectively)
	 * 
	 * <p> A ScoringPosition object may be passed into a {@link frc.robot.commands.scoring.SetScoringPosition SetScoringPosition}
	 * command to set both mechanisms at the same time
	 */
	public record ScoringPosition(double armAngle_deg, double shooterSpeed_rpm) {};

	// experimental scoring data (unused: formulas from linear regression are better)
	public static final HashMap<Double, ScoringPosition> scoringData = new HashMap<>();
	static {
		scoringData.put(1.445, new ScoringPosition(-1, 3200));
		scoringData.put(1.718, new ScoringPosition(3, 3200));
		scoringData.put(1.817, new ScoringPosition(5, 3250));
		scoringData.put(2.048, new ScoringPosition(8, 3250));
		scoringData.put(2.426, new ScoringPosition(9, 3250));
		scoringData.put(2.874, new ScoringPosition(16, 3350));
		scoringData.put(3.227, new ScoringPosition(19, 3450));
	}

	// presets
	public static final ScoringPosition ampPosition = new ScoringPosition(93, 1750);
	public static final ScoringPosition subwooferPosition = new ScoringPosition(-1.5, 3250);
	public static final ScoringPosition carryPosition = new ScoringPosition(40, 0);
	public static final ScoringPosition lowCarryPosition = new ScoringPosition(4, 0);
	public static final ScoringPosition passPosition = new ScoringPosition(1, 3000);
	public static final ScoringPosition lowPassPosition = new ScoringPosition(54, 2750);
}
