package frc.robot.subsystems;

import frc.robot.constants.ScoringConstants;
import frc.robot.constants.ScoringConstants.ScoringPosition;
import frc.robot.constants.VisionConstants;
import frc.robot.io.FieldVisionIO;
import frc.robot.io.FieldVisionIOInputsAutoLogged;
import frc.robot.io.PieceVisionIO;
import frc.robot.io.PieceVisionIOInputsAutoLogged;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.stream.Collectors;

public class Vision extends SubsystemBase {
	private FieldVisionIO fieldIO = new FieldVisionIO();
	private PieceVisionIO pieceIO = new PieceVisionIO();
	public FieldVisionIOInputsAutoLogged fieldInputs = new FieldVisionIOInputsAutoLogged();
	public PieceVisionIOInputsAutoLogged pieceInputs = new PieceVisionIOInputsAutoLogged();
	private NetworkTableEntry priorityIdEntry = NetworkTableInstance.getDefault().getTable("limelight-fv")
		.getEntry("priorityid");

	public Vision() {}

	@Override
	public void periodic() {
		fieldIO.updateInputs(fieldInputs);
		pieceIO.updateInputs(pieceInputs);
		Logger.processInputs(getName() + "/field", fieldInputs);
		Logger.processInputs(getName() + "/piece", pieceInputs);
	}

	public void setPriorityId(int id) {
		priorityIdEntry.setNumber(id);
	}

	public double findDistance() {
		double targetOffsetAngle_Vertical = fieldInputs.ty;
		double angleToGoalDegrees = VisionConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

		double distance = (VisionConstants.goalHeight - VisionConstants.LensHeight) / Math.tan(angleToGoalRadians);

		return distance;
	}

	public ScoringPosition estimateScoringPosition_math() {
		double distance = Math.hypot(fieldInputs.targetspaceTranslationX_m, fieldInputs.targetspaceTranslationZ_m);

		double angle = -20 + (14.9 * distance) + (-.889 * Math.pow(distance, 2));
		double speed = 2515 + (990 * distance) + (-473 * Math.pow(distance, 2)) + (79.4 * Math.pow(distance, 3));

		Logger.recordOutput("Vision/targetPosition_deg", angle);
		Logger.recordOutput("Vision/targetPosition_rpm", speed);

		return new ScoringPosition(angle, speed);
	}

	public ScoringPosition estimateScoringPosition_map() {
		double distance = Math.hypot(fieldInputs.targetspaceTranslationX_m, fieldInputs.targetspaceTranslationZ_m);

		// sort the keys with stream methods then collect into a set
		ArrayList<Double> keys = ScoringConstants.scoringMap.keySet().stream().sorted()
			.collect(Collectors.toCollection(ArrayList::new));
		// get closest value
		double closestKey_m = keys.stream().min(Comparator.comparing(i -> Math.abs(i - distance))).orElseThrow()
			.doubleValue();

		// instantiate bounds to be used for linear interpolation
		double upperKey_m, lowerKey_m;
		if (closestKey_m < distance) {
			// the closest value is less than the actual distance, set it as the lower bound and set the upper bound as the
			// next one up
			lowerKey_m = closestKey_m;
			upperKey_m = keys.get(keys.indexOf(closestKey_m) + 1);
		} else {
			// opposite if the closest value is above the actual distance
			upperKey_m = closestKey_m;
			lowerKey_m = keys.get(keys.indexOf(closestKey_m) - 1);
		}
		double interpolationDistance = (distance - lowerKey_m) / (upperKey_m - lowerKey_m);

		var lowerPosition = ScoringConstants.scoringMap.get(lowerKey_m);
		var upperPosition = ScoringConstants.scoringMap.get(upperKey_m);

		// interpolate between the two positions to get the estimated angle and speed
		double angle = MathUtil.interpolate(lowerPosition.armAngle_deg(), upperPosition.armAngle_deg(),
			interpolationDistance);
		double speed = MathUtil.interpolate(lowerPosition.shooterSpeed_rpm(), upperPosition.shooterSpeed_rpm(),
			interpolationDistance);

		Logger.recordOutput("Vision/targetPosition", new double[] {angle, speed});

		return new ScoringPosition(angle, speed);
	}
}

/*
 * The point of this subsystem is to take all the inputs from VisionIO and log them.
 * From there we can take values from here and apply them elsewhere
 */