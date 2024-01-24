package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// based on https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/frc/robot/FieldConstants.java,
// https://github.com/TeamSCREAMRobotics/4522_2024Competition/blob/main/src/main/java/frc/robot/Constants.java

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters,
 * and sets of corners start in the lower left moving clockwise.
 * <p>
 * NOTE staging locations are indexed going away from AMP wall
 * <p>
 * All translations and poses are stored with the origin at the rightmost point on the BLUE ALLIANCE wall.
 * <p>
 * length refers to the x-direction (as described by wpilib), width refers to the y-direction (as described by wpilib)
 */
public final class FieldConstants {
	public static final double fieldLength_m = Units.inchesToMeters(651.223);
	public static final double fieldWidth_m = Units.inchesToMeters(323.277);

	public static final double startingLineX_m = Units.inchesToMeters(74.111);
	public static final double podiumX_m = Units.inchesToMeters(126.75);
	public static final double wingX_m = Units.inchesToMeters(229.201);
	public static final double centerLineX_m = fieldLength_m / 2;

	// public static final Translation2d[] centerStagedNotes = new Translation2d[]{}; // todo:
	// public static final Translation2d[] wingStagedNotes = new Translation2d[]{}; // todo:

	public static final Translation2d blueAmpCenter_m = new Translation2d(Units.inchesToMeters(72.455),
		Units.inchesToMeters(322.996));
	public static final Translation2d redAmpCenter_m = flipFieldPosition(blueAmpCenter_m);

	public static final Translation2d blueSpeakerOpening_m = new Translation2d(0.0,
		fieldWidth_m - Units.inchesToMeters(104.0));
	public static final Translation2d redSpeakerOpening_m = flipFieldPosition(blueSpeakerOpening_m);

	public static Translation2d flipFieldPosition(Translation2d pos) {
		return new Translation2d(fieldLength_m - pos.getX(), pos.getY());
	}
}
