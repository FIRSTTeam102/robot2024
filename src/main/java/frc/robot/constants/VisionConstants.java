package frc.robot.constants;

public class VisionConstants {

	/* AprilTagVision */
	public static final double poseError_m = 1; // comparing visionPose to pose
	public static final double botpose_fieldOffsetX_m = 0.025; // realife offset to pathplanner app
	/* ObjectDetectionVision */
	public static final double visionRotateKp = 1.90;
	public static final double visionRotateKd = 0.5; // 1.5

	// public static final double AprilTagRotateKp = 1.2;
	// public static final double AprilTagRotateKd = 0.34;

	public static final double crosshairGamePieceBoundRotateX_rad = Math.toRadians(1.5);

	public static final int redSpeakerTag = 4;
	public static final int blueSpeakerTag = 7;

	public static final double limelightMountAngleDegrees = 15; // placeholder
	public static final double LensHeight = 26; // placeholder
	public static final double goalHeight = 53.63; // Speaker tag center height

	public static final double limelightXOffset = 12; // offset the limelight is from the center of the robot

}
