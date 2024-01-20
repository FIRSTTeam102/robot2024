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
public class FieldVisionIO {
	@AutoLog
	public static class FieldVisonIOInputs {
		// basic limelight configuration settings
		public long fieldVisionPipeline = 0;
		public boolean fieldVisionTarget = false;
		public long fieldVisionTargetAprilTag = 0;

		// basic limelight target info
		public double fieldVisionCrosshairToTargetErrorX_rad = 0.0;
		public double fieldVisionCrosshairToTargetErrorY_rad = 0.0;
		public double fieldVisionTargetArea = 0.0;

		// Field orientations
		// "Where am I in relation to my target apriltag"
		public double fieldVisionBotpose_TargetspaceTranslationX_m = 0.0;
		public double fieldVisionBotpose_TargetspaceTranslationY_m = 0.0;
		public double fieldVisionBotpose_TargetspaceTranslationZ_m = 0.0;
		public double fieldVisionBotpose_TargetspaceRotationX_rad = 0.0;
		public double fieldVisionBotpose_TargetspaceRotationY_rad = 0.0;
		public double fieldVisionBotpose_TargetspaceRotationZ_rad = 0.0;
		// "Where am I on the field"
		public double fieldVisionBotpose_FieldspaceTranslationX_m = 0.0;
		public double fieldVisionBotpose_FieldspaceTranslationY_m = 0.0;
		public double fieldVisionBotpose_FieldspaceTranslationZ_m = 0.0;
		public double fieldVisionBotpose_FieldspaceRotationX_rad = 0.0;
		public double fieldVisionBotpose_FieldspaceRotationY_rad = 0.0;
		public double fieldVisionBotpose_FieldspaceRotationZ_rad = 0.0;

		public double fieldVisionBotpose_Latency_s = 0.0; // Used in the estimation of our pose whilst considering lag
	}

	// methods to get information from LL
	private double tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
	private double cl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0);

	// Basic targeting info
	private NetworkTable tableFieldVision = NetworkTableInstance.getDefault().getTable("limelight-fv");

	private NetworkTableEntry pipelineEntryFieldVision = tableFieldVision.getEntry("pipeline");
	private NetworkTableEntry tvEntryFieldVision = tableFieldVision.getEntry("tv");
	private NetworkTableEntry tidEntryFieldVision = tableFieldVision.getEntry("tid");

	private NetworkTableEntry txEntryFieldVision = tableFieldVision.getEntry("tx");
	private NetworkTableEntry tyEntryFieldVision = tableFieldVision.getEntry("ty");
	private NetworkTableEntry taEntryFieldVision = tableFieldVision.getEntry("ta");

	// 3D data
	private NetworkTableEntry botpose_targetspaceEntryFieldVision = tableFieldVision.getEntry("botpose_targetspace");
	private double[] botpose_targetspaceCacheFieldVision = new double[6]; // The array that will hold all the position

	private NetworkTableEntry botpose_wpiblueEntryFieldVision = tableFieldVision.getEntry("botpose_wpiblue");
	private double[] botpose_wpiblueCacheFieldVision = new double[7];

	public void updateInputs(FieldVisonIOInputs inputs) {
		inputs.fieldVisionPipeline = pipelineEntryFieldVision.getNumber(inputs.fieldVisionPipeline).intValue();
		inputs.fieldVisionTarget = tvEntryFieldVision.getDouble(0) == 1;
		inputs.fieldVisionTargetAprilTag = tidEntryFieldVision.getNumber(inputs.fieldVisionTargetAprilTag).intValue();
		inputs.fieldVisionCrosshairToTargetErrorX_rad = Math
			.toRadians(txEntryFieldVision.getDouble(inputs.fieldVisionCrosshairToTargetErrorX_rad));
		inputs.fieldVisionCrosshairToTargetErrorY_rad = Math
			.toRadians(tyEntryFieldVision.getDouble(inputs.fieldVisionCrosshairToTargetErrorX_rad));
		inputs.fieldVisionTargetArea = taEntryFieldVision.getDouble(inputs.fieldVisionTargetArea);

		// Log all input
		botpose_targetspaceCacheFieldVision = botpose_targetspaceEntryFieldVision
			.getDoubleArray(botpose_targetspaceCacheFieldVision);
		if (botpose_targetspaceCacheFieldVision.length > 0) {
			inputs.fieldVisionBotpose_TargetspaceTranslationX_m = botpose_targetspaceCacheFieldVision[0];
			inputs.fieldVisionBotpose_TargetspaceTranslationY_m = botpose_targetspaceCacheFieldVision[1];
			inputs.fieldVisionBotpose_TargetspaceTranslationZ_m = botpose_targetspaceCacheFieldVision[2];
			inputs.fieldVisionBotpose_TargetspaceRotationX_rad = Math.toRadians(botpose_targetspaceCacheFieldVision[3]);
			inputs.fieldVisionBotpose_TargetspaceRotationY_rad = Math.toRadians(botpose_targetspaceCacheFieldVision[4]);
			inputs.fieldVisionBotpose_TargetspaceRotationZ_rad = Math.toRadians(botpose_targetspaceCacheFieldVision[5]);
			inputs.fieldVisionBotpose_Latency_s = botpose_wpiblueCacheFieldVision[6] / 1000;
		} else
			DriverStation.reportWarning("Invalid botpose array from limelight", true);

		botpose_wpiblueCacheFieldVision = botpose_wpiblueEntryFieldVision.getDoubleArray(botpose_wpiblueCacheFieldVision);
		inputs.fieldVisionBotpose_FieldspaceTranslationX_m = botpose_wpiblueCacheFieldVision[0];
		inputs.fieldVisionBotpose_FieldspaceTranslationY_m = botpose_wpiblueCacheFieldVision[1];
		inputs.fieldVisionBotpose_FieldspaceTranslationZ_m = botpose_wpiblueCacheFieldVision[2];
		inputs.fieldVisionBotpose_FieldspaceRotationX_rad = Math.toRadians(botpose_wpiblueCacheFieldVision[3]);
		inputs.fieldVisionBotpose_FieldspaceRotationY_rad = Math.toRadians(botpose_wpiblueCacheFieldVision[4]);
		inputs.fieldVisionBotpose_FieldspaceRotationZ_rad = Math.toRadians(botpose_wpiblueCacheFieldVision[5]);
		inputs.fieldVisionBotpose_Latency_s = botpose_wpiblueCacheFieldVision[6] / 1000;
	}
}
