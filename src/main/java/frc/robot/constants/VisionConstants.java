package frc.robot.constants;

public class VisionConstants {
	/* AprilTagVision */
	public static final double poseError_m = 1; // comparing visionPose to pose
	public static final double botpose_fieldOffsetX_m = 0.025; // realife offset to pathplanner app
	/* ObjectDetectionVision */
	public static final double gamePieceRotateKp = 1.2;
	public static final double gamePieceRotateKd = 0.34;
	public static final double AprilTagRotateKp = 1.2;
	public static final double AprilTagRotateKd = 0.34;
	public static final double crosshairGamePieceBoundRotateX_rad = Math.toRadians(1.0);
	public static final int redSpeakerTag = 4;
	public static final int blueSpeakerTag = 7;
}
