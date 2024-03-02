package frc.robot.subsystems;

import static frc.robot.constants.AutoConstants.*;
import static frc.robot.constants.SwerveConstants.*;

import frc.robot.Robot;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIOReal;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.util.LocalADStarAK;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Swerve extends SubsystemBase {
	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

	public SwerveModule[] modules;
	public SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(),
		new SwerveModulePosition(), new SwerveModulePosition()};
	public SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(),
		new SwerveModuleState(), new SwerveModuleState()};

	public SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
		kinematics, new Rotation2d(), modulePositions, new Pose2d());

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public Pose2d[] modulePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
	public Field2d fieldSim = new Field2d();

	@Getter
	private Translation2d centerRotation = new Translation2d(0, 0); // position the robot rotates around

	// used for pose estimation
	private Timer timer = new Timer();

	public GyroIO gyroIO;
	public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	private Vision vision;
	private double translationY;
	private double translationX;

	public Swerve(GyroIO gyroIO, Vision vision) {
		modules = new SwerveModule[moduleConstants.length];
		int moduleNumber = 0;
		for (var mod : moduleConstants) {
			modules[moduleNumber] = new SwerveModule(moduleNumber, Robot.isReal()
				? new SwerveModuleIOReal(mod, moduleNumber)
				: new SwerveModuleIOSim());
			moduleNumber++;
		}

		this.vision = vision;

		this.gyroIO = gyroIO;
		zeroYaw();

		timer.reset();
		timer.start();

		SmartDashboard.putData("Field", fieldSim);

		// configure pathplanner
		AutoBuilder.configureHolonomic(
			this::getPose,
			this::resetOdometry,
			() -> kinematics.toChassisSpeeds(getStates()),
			this::setChasisSpeeds,
			new HolonomicPathFollowerConfig(
				new PIDConstants(autoDriveKp, autoDriveKi, autoDriveKd),
				new PIDConstants(autoAngleKp, autoAngleKi, autoAngleKd),
				maxVelocity_mps,
				Math.hypot(trackWidth_m / 2.0, trackWidth_m / 2.0),
				new ReplanningConfig()),
			() -> {
				var alliance = DriverStation.getAlliance();
				return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
			},
			this);
		Pathfinding.setPathfinder(new LocalADStarAK());

		// this.vision = vision;
	}

	/** update and return states */
	public SwerveModuleState[] getStates() {
		for (SwerveModule mod : modules) {
			moduleStates[mod.moduleNumber] = mod.getState();
		}
		return moduleStates;
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		setModuleStates(desiredStates, false, false);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean forceAngle) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxVelocity_mps);
		for (SwerveModule mod : modules)
			mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, forceAngle);
	}

	public void setChasisSpeeds(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerRotation);
		setModuleStates(states);
	}

	/** @return the estimated pose of the robot on the field */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public SwerveModulePosition[] getPositions() {
		for (SwerveModule mod : modules) {
			modulePositions[mod.moduleNumber] = mod.getPosition();
		}
		return modulePositions;
	}

	/**
	 * sets the odometry of the robot to the specified PathPlanner state
	 * 
	 * this should only be done when the rotation of the robot is known
	 * (like at the start of an autonomous path)
	 */
	public void resetOdometry(Pose2d pose) {
		poseEstimator.resetPosition(getYaw(), getPositions(), pose);
	}

	public void resetModuleOffsets() {
		for (var mod : modules)
			mod.io.setOffset(0);
	}

	/** @return gyro yaw including code offset */
	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(gyroInputs.yaw_deg);
	}

	public void zeroYaw() {
		gyroIO.setYaw(0);
	}

	public double getTilt_rad() {
		var yaw = getYaw().getRadians();
		return (gyroInputs.pitch_rad * Math.sin(yaw)) + (gyroInputs.roll_rad * -Math.cos(yaw));
	}

	public void setBrakeMode(boolean enable) {
		for (SwerveModule mod : modules)
			mod.setDriveBrakeMode(enable);
	}

	public void setCenterRotation(double x, double y) {
		centerRotation = new Translation2d(x, y);
	}

	/**
	 * sets the velocity of the robot
	 * 
	 * <p>if robot oriented, +x is forward, +y is left, +rotation is CCW;
	 * <p>if field oriented, the origin of the field is the lower left corner (corner of the field to the driver's right);
	 * for rotation zero is away from the driver, positive is CCW
	 */
	public void drive(Translation2d translation_mps, double rotation_radps, boolean fieldRelative) {
		SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(
					translation_mps.getX(),
					translation_mps.getY(),
					rotation_radps,
					getYaw())
				: new ChassisSpeeds(
					translation_mps.getX(),
					translation_mps.getY(),
					rotation_radps),
			centerRotation);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity_mps);

		for (SwerveModule mod : modules) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false, false);
		}
	}

	public void stop() {
		chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerRotation);
		setModuleStates(states);
	}

	public SwerveModuleState[] getXStanceStates() {
		var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0), new Translation2d(0, 0));
		states[0].angle = new Rotation2d(3 * Math.PI / 2 - Math.atan(trackWidth_m / wheelBase_m));
		states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[3].angle = new Rotation2d(Math.PI / 2 - Math.atan(trackWidth_m / wheelBase_m));
		return states;
	}

	private double visionSeenCount = 0;

	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Gyro", gyroInputs);
		for (SwerveModule mod : modules) {
			mod.periodic();

			modulePositions[mod.moduleNumber] = mod.getPosition();
			moduleStates[mod.moduleNumber] = mod.getState();
		}
		Logger.recordOutput("Swerve/States", moduleStates);

		// update odometry
		poseEstimator.updateWithTime(timer.get(), getYaw(), modulePositions);
		var pose = poseEstimator.getEstimatedPosition();
		translationY = pose.getY();
		translationX = pose.getX();

		// Rotation2d rotation = new Rotation2d(vision.fieldInputs.fieldspaceRotationX_rad);
		// double translationX = vision.fieldInputs.fieldspaceTranslationY_m;
		// double translationY = vision.fieldInputs.fieldspaceTranslationX_m;
		// Pose2d visionPose = new Pose2d(translationX, translationY, rotation);
		// poseEstimator.addVisionMeasurement(visionPose, timer.get() - vision.fieldInputs.fieldspaceTotalLatency_s);
		// todo: estimate without using gyro?

		// Every 0.02s, updating pose2d
		if (vision.fieldInputs.hasTarget == true) {
			visionSeenCount++;
			if (visionSeenCount > 2) { // The if statement is used to eliminate "hallucinations". Schizophrenic robot smh
				var visionPose = new Pose2d(vision.fieldInputs.fieldspaceTranslationX_m,
					vision.fieldInputs.fieldspaceTranslationY_m,
					new Rotation2d(vision.fieldInputs.fieldspaceRotationX_rad)); // continuation of lines 259-261. Create a new
																																				// pose2d
				// using X,Y and rotation
				Logger.recordOutput("Odometry/VisionPose", visionPose); // log the odometry to advantage kit
				var distance = Math.hypot( // use pythagorean theorem to find the distance from the last position
					Math.abs(translationX - vision.fieldInputs.fieldspaceTranslationX_m),
					Math.abs(translationY - vision.fieldInputs.fieldspaceTranslationY_m));
				poseEstimator.addVisionMeasurement(visionPose,
					timer.get() - vision.fieldInputs.fieldspaceTotalLatency_s); // remove lag from the time in the calculation of
																																			// estimated pose
				VecBuilder.fill(distance / 2, distance / 2, 100);
			}
		} else
			visionSeenCount = 0;
		Logger.recordOutput("Odometry/Robot", pose);
		// update field pose
		for (int i = 0; i < modules.length; i++) {
			modulePoses[i] = new Pose2d(
				moduleTranslations[i]
					.rotateBy(getYaw())
					.plus(pose.getTranslation()),
				moduleStates[i].angle
					.plus(pose.getRotation())
					// show movement direction instead of physical module direction since it's optimized
					.plus(Rotation2d.fromRadians(moduleStates[i].speedMetersPerSecond < 0 ? Math.PI : 0)));
		}
		fieldSim.setRobotPose(pose);
		fieldSim.getObject("Swerve Modules").setPoses(modulePoses);
	}

	// until advantagescope wpilog imports are fixed, use built in data log to gather data

	// public final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
	// new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
	// new SysIdRoutine.Mechanism(
	// (voltage) -> {
	// var volts = voltage.in(Units.Volts);
	// for (var mod : modules)
	// mod.runDriveCharacterization(volts);
	// },
	// null,
	// this));
	public final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(
			(voltage) -> {
				var volts = voltage.in(Units.Volts);
				for (var mod : modules)
					mod.runDriveCharacterization(volts);
			},
			log -> {
				for (int i = 0; i < modules.length; i++)
					log.motor("drive" + i)
						.voltage(Units.Volts.of(modules[i].inputs.driveAppliedVoltage_V))
						.linearPosition(Units.Meters.of(modules[i].inputs.driveDistance_m))
						.linearVelocity(Units.MetersPerSecond.of(modules[i].inputs.driveVelocity_mps));
			},
			this, "swerve-drive"));

	// public final SysIdRoutine angleSysIdRoutine = new SysIdRoutine(
	// new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
	// new SysIdRoutine.Mechanism(
	// (voltage) -> {
	// var volts = voltage.in(Units.Volts);
	// for (var mod : modules)
	// mod.runAngleCharacterization(volts);
	// },
	// null,
	// this));
	public final SysIdRoutine angleSysIdRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(),
		new SysIdRoutine.Mechanism(
			(voltage) -> {
				var volts = voltage.in(Units.Volts);
				for (var mod : modules)
					mod.runAngleCharacterization(volts);
			},
			log -> {
				for (int i = 0; i < modules.length; i++)
					log.motor("angle" + i)
						.voltage(Units.Volts.of(modules[i].inputs.angleAppliedVoltage_V))
						.linearPosition(Units.Meters.of(modules[i].inputs.angleAbsolutePosition_rad))
						.linearVelocity(Units.MetersPerSecond.of(modules[i].inputs.angleVelocity_radps));
			},
			this, "swerve-angle"));
}
