package frc.robot.io;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.AutoLog;

public class FieldVisionIO {
	@AutoLog
	public class FieldVisionIOInputs {
		public int pipeline = 0;
		public boolean hasTarget = false;
		public int targetAprilTag = 0;

		public double crosshairToTargetErrorX_rad = 0.0;
		public double crosshairToTargetErrorY_rad = 0.0;
		public double targetArea = 0.0;

		public double targetspaceTranslationX_m = 0.0;
		public double targetspaceTranslationY_m = 0.0;
		public double targetspaceTranslationZ_m = 0.0;
		public double targetspaceRotationX_rad = 0.0;
		public double targetspaceRotationY_rad = 0.0;
		public double targetspaceRotationZ_rad = 0.0;

		public double fieldspaceTranslationX_m = 0.0;
		public double fieldspaceTranslationY_m = 0.0;
		public double fieldspaceTranslationZ_m = 0.0;
		public double fieldspaceRotationX_rad = 0.0;
		public double fieldspaceRotationY_rad = 0.0;
		public double fieldspaceRotationZ_rad = 0.0;

		public double fieldspaceTotalLatency_s = 0.0;
	}

	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fv");

	private NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
	private NetworkTableEntry tvEntry = table.getEntry("tv");
	private NetworkTableEntry tidEntry = table.getEntry("tid");

	private NetworkTableEntry txEntry = table.getEntry("tx");
	private NetworkTableEntry tyEntry = table.getEntry("ty");
	private NetworkTableEntry taEntry = table.getEntry("ta");

	private NetworkTableEntry clEntry = table.getEntry("cl");
	private NetworkTableEntry tlEntry = table.getEntry("tl");

	private NetworkTableEntry targetspaceEntry = table.getEntry("targetspace");
	private double[] targetspaceCache = new double[6]; // array that will hold all the positions

	private NetworkTableEntry botpose_wpiblueEntry = table.getEntry("botpose_wpiblue");
	private double[] botpose_wpiblueCache = new double[7];

	public void updateInputs(FieldVisionIOInputs inputs) {
		inputs.pipeline = pipelineEntry.getNumber(inputs.pipeline).intValue();
		inputs.hasTarget = tvEntry.getDouble(0) == 1;
		inputs.targetAprilTag = tidEntry.getNumber(inputs.targetAprilTag).intValue();
		inputs.crosshairToTargetErrorX_rad = Math.toRadians(txEntry.getDouble(inputs.crosshairToTargetErrorX_rad));
		inputs.crosshairToTargetErrorY_rad = Math.toRadians(tyEntry.getDouble(inputs.crosshairToTargetErrorX_rad));
		inputs.targetArea = taEntry.getDouble(inputs.targetArea);

		targetspaceCache = targetspaceEntry.getDoubleArray(targetspaceCache);
		if (targetspaceCache.length > 0) {
			inputs.targetspaceTranslationX_m = targetspaceCache[0];
			inputs.targetspaceTranslationY_m = targetspaceCache[1];
			inputs.targetspaceTranslationZ_m = targetspaceCache[2];
			inputs.targetspaceRotationX_rad = Math.toRadians(targetspaceCache[3]);
			inputs.targetspaceRotationY_rad = Math.toRadians(targetspaceCache[4]);
			inputs.targetspaceRotationZ_rad = Math.toRadians(targetspaceCache[5]);
		} else
			DriverStation.reportWarning("invalid targetspace array from limelight", true);

		if (botpose_wpiblueCache.length > 0) {
			botpose_wpiblueCache = botpose_wpiblueEntry.getDoubleArray(botpose_wpiblueCache);
			inputs.fieldspaceTranslationX_m = botpose_wpiblueCache[0];
			inputs.fieldspaceTranslationY_m = botpose_wpiblueCache[1];
			inputs.fieldspaceTranslationZ_m = botpose_wpiblueCache[2];
			inputs.fieldspaceRotationX_rad = Math.toRadians(botpose_wpiblueCache[3]);
			inputs.fieldspaceRotationY_rad = Math.toRadians(botpose_wpiblueCache[4]);
			inputs.fieldspaceRotationZ_rad = Math.toRadians(botpose_wpiblueCache[5]);
			inputs.fieldspaceTotalLatency_s = botpose_wpiblueCache[6] / 1000;
		} else {
			DriverStation.reportWarning("invalid wpiblue array from limelight", true);
			inputs.fieldspaceTotalLatency_s = (tlEntry.getDouble(0) - clEntry.getDouble(0)) / 1000;
		}
	}
}
