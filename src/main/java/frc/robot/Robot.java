package frc.robot;

import static frc.robot.constants.Constants.*;

import frc.robot.constants.BuildConstants;
import frc.robot.constants.LightsConstants;
import frc.robot.constants.ScoringConstants;
import frc.robot.subsystems.Lights;
import frc.robot.util.AutoSetterTunableNumber.AutoSetterTunableNumberManager;
import frc.robot.util.VirtualSubsystem;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.revrobotics.REVPhysicsSim;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends LoggedRobot {
	private Command autonomousCommand;
	private RobotContainer robotContainer;

	Alliance alliance;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		if (Robot.isSimulation())
			DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

		Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
			case 0 -> "Clean";
			case 1 -> "Dirty";
			default -> "Unknown";
		});

		switch (robotMode) {
			case Replay -> {
				setUseTiming(false); // run as fast as possible
				String logPath = LogFileUtil.findReplayLog(); // pull replay log from AdvantageScope (or prompt the user)
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
			}
			case Active -> {
				Logger.addDataReceiver(new WPILOGWriter("/media/sda/logs/")); // log to a usb stick
				Logger.addDataReceiver(new NT4Publisher()); // publish data to NetworkTables
				if (isReal()) {
					new PowerDistribution(pdhId, ModuleType.kRev); // enables power distribution logging
				} else {
					// sim
				}
			}
		}

		PathPlannerLogging.setLogCurrentPoseCallback((pose) -> Logger.recordOutput("Odometry/PPCurrentPose", pose));
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> Logger.recordOutput("Odometry/PPTargetPose", pose));
		PathPlannerLogging.setLogActivePathCallback(
			(poses) -> Logger.recordOutput("Odometry/PPTrajectory", poses.toArray(new Pose2d[poses.size()])));

		// if (tuningMode)
		// Logger.registerURCL(URCL.startExternal());

		Logger.start();

		robotContainer = RobotContainer.getInstance();

		// disable LiveWindow telemetry in favor of AdvantageKit to reduce processing each tick
		LiveWindow.disableAllTelemetry();

		if (!tuningMode)
			PPLibTelemetry.enableCompetitionMode();

		// robotContainer.swerve.zeroYaw();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		if (tuningMode)
			AutoSetterTunableNumberManager.periodic();

		VirtualSubsystem.periodicAll();

		/*
		 * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
		 * commands, running already-scheduled commands, removing finished or interrupted commands,
		 * and running subsystem periodic() methods. This must be called from the robot's periodic
		 * block in order for anything in the Command-based framework to work.
		 */
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		robotContainer.swerve.stop();
		Lights.setStatus(LightsConstants.Mode.Disabled);
	}

	@Override
	public void disabledPeriodic() {
		robotContainer.updateOIAlert();
	}

	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}

		alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;

		// for limelight shooting targeting
		switch (alliance) {
			case Red -> robotContainer.vision.setPriorityId(4);
			case Blue -> robotContainer.vision.setPriorityId(7);
		}

		Lights.setStatus(LightsConstants.Mode.Auto);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		/*
		 * This makes sure that the autonomous stops running when teleop starts running. If you want the
		 * autonomous to continue until interrupted by another command, remove this line or comment it
		 * out.
		 */
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		robotContainer.arm.setPosition(ScoringConstants.carryPosition.armAngle_deg());
		robotContainer.arm.climbStartup().schedule();
		alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;

		// for limelight shooting targeting
		switch (alliance) {
			case Red -> {
				robotContainer.vision.setPriorityId(4);
				Lights.setStatus(LightsConstants.Mode.TeleopRED);
			}
			case Blue -> {
				robotContainer.vision.setPriorityId(7);
				Lights.setStatus(LightsConstants.Mode.TeleopBLUE);
			}
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		robotContainer.checkRumble();
		if (robotContainer.intake.isHoldingNote())
			Lights.setStatus(LightsConstants.Mode.HaveANote);
		else
			Lights.setDefaultStatus();
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		REVPhysicsSim.getInstance().run();
	}

	public static boolean isBlue() {
		// defaults to blue when unknown
		return !DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() != DriverStation.Alliance.Red;
	}

}
