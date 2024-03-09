package frc.robot.constants;

import java.util.HashMap;

public final class ScoringConstants {
	public record ScoringPosition(double armAngle_deg, double shooterSpeed_rpm) {};

	public static final HashMap<Double, ScoringPosition> scoringMap = new HashMap<>();
	static {
		scoringMap.put(1.445, new ScoringPosition(-1, 3200));
		scoringMap.put(1.718, new ScoringPosition(3, 3200));
		scoringMap.put(1.817, new ScoringPosition(5, 3250));
		scoringMap.put(2.048, new ScoringPosition(8, 3250));
		scoringMap.put(2.426, new ScoringPosition(9, 3250));
		scoringMap.put(2.874, new ScoringPosition(16, 3350));
		scoringMap.put(3.227, new ScoringPosition(19, 3450));
	}
}
