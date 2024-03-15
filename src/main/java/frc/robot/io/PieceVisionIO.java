// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public class PieceVisionIO {
	@AutoLog
	public static class PieceVisionIOInputs {
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

		// This stuff is useless, but is still stuf that will be logged bc why not
		public double fieldspaceTranslationX_m = 0.0;
		public double fieldspaceTranslationY_m = 0.0;
		public double fieldspaceTranslationZ_m = 0.0;
		public double fieldspaceRotationX_rad = 0.0;
		public double fieldspaceRotationY_rad = 0.0;
		public double fieldspaceRotationZ_rad = 0.0;

		public double fieldspaceTotalLatency_s = 0.0;
	}

	// Network table declarations
	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-gpv");

	private NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
	private NetworkTableEntry tvEntry = table.getEntry("tv");
	private NetworkTableEntry tidEntry = table.getEntry("tid");

	private NetworkTableEntry txEntry = table.getEntry("tx");
	private NetworkTableEntry tyEntry = table.getEntry("ty");
	private NetworkTableEntry taEntry = table.getEntry("ta");

	private NetworkTableEntry clEntry = table.getEntry("cl");
	private NetworkTableEntry tlEntry = table.getEntry("tl");

	private NetworkTableEntry targetspaceEntry = table.getEntry("botpose_targetspace");
	private double[] targetspaceCache = new double[6]; // array that will hold all the positions

	public void updateInputs(PieceVisionIOInputs inputs) {
		inputs.pipeline = pipelineEntry.getNumber(inputs.pipeline).intValue();
		inputs.hasTarget = tvEntry.getDouble(0) == 1;
		inputs.targetAprilTag = tidEntry.getNumber(inputs.targetAprilTag).intValue();
		inputs.crosshairToTargetErrorX_rad = Math.toRadians(txEntry.getDouble(inputs.crosshairToTargetErrorX_rad));
		inputs.crosshairToTargetErrorY_rad = Math.toRadians(tyEntry.getDouble(inputs.crosshairToTargetErrorY_rad));
		inputs.targetArea = taEntry.getDouble(inputs.targetArea);

		targetspaceCache = targetspaceEntry.getDoubleArray(targetspaceCache);
		if (targetspaceCache.length > 1) {
			inputs.targetspaceTranslationX_m = targetspaceCache[0];
			inputs.targetspaceTranslationY_m = targetspaceCache[1];
			inputs.targetspaceTranslationZ_m = targetspaceCache[2];
			inputs.targetspaceRotationX_rad = Math.toRadians(targetspaceCache[3]);
			inputs.targetspaceRotationY_rad = Math.toRadians(targetspaceCache[4]);
			inputs.targetspaceRotationZ_rad = Math.toRadians(targetspaceCache[5]);
		} else
			DriverStation.reportWarning("invalid targetspace array from limelight", true);
	}
}
