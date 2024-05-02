package frc.robot;

import static frc.robot.constants.Constants.OperatorConstants.*;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.ShuffleboardConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.ScoringConstants.ScoringPosition;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SystemAlerter;
import frc.robot.subsystems.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.ControllerUtil;

import frc.robot.commands.arm.AutoClimb;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.intake.IntakeWithArm;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.scoring.SetScoringPosition;
import frc.robot.commands.shooter.SetShooterVelocity;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.swerve.SwerveAngleOffsetCalibration;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.commands.vision.GamePieceVision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private static RobotContainer instance = null;

	public static RobotContainer getInstance() {

		if (instance == null)
			instance = new RobotContainer();
		return instance;
	}

	private final CommandXboxController driverController = new CommandXboxController(driverControllerPort);
	private final CommandXboxController operatorController = new CommandXboxController(operaterControllerPort);
	private final CommandXboxController testController = new CommandXboxController(testControllerPort);
	private final CommandXboxController demoController = new CommandXboxController(demoControllerPort);

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("auto routine");

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/* subsystems */
	public final Vision vision = new Vision();
	public final Arm arm = new Arm();
	public final Intake intake = new Intake();
	public final Swerve swerve = new Swerve(gyro, vision);
	public final Shooter shooter = new Shooter();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		new SystemAlerter();

		configureBindings();

		autoChooser.addOption("nothing", Commands.none());

		final var driveTab = Shuffleboard.getTab(ShuffleboardConstants.driveTab);
		var delayEntry = driveTab.add("Delay Auto?", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(3, 1)
			.withPosition(0, 6)
			.getEntry();
		var noteStartEntry = driveTab.add("Starting with Note?", false).withWidget(BuiltInWidgets.kToggleSwitch)
			.withSize(3, 1).withPosition(0, 7).getEntry();
		driveTab.add("auto routine", autoChooser.getSendableChooser())
			.withSize(4, 1).withPosition(0, 5);
		driveTab.add("alerts", Alert.getAlertsSendable())
			.withSize(5, 4).withPosition(4, 5).withWidget(Alert.widgetName);
		// driveTab.add("camera", SendableCameraWrapper.wrap("limelight-stream", "http://10.1.2.12:5800/stream.mjpg"))
		// .withProperties(Map.of("show crosshair", false, "show controls", false))
		// .withWidget(BuiltInWidgets.kCameraStream)
		// .withSize(11, 5)
		// .withPosition(0, 0);
		try {
			var camera = CameraServer.startAutomaticCapture();
			camera.setResolution(240, 60);
			camera.setFPS(10);
			// camera.setPixelFormat(PixelFormat.kMJPEG);
			driveTab.add("camera", camera).withWidget(BuiltInWidgets.kCameraStream).withProperties(Map.of("comp", 50))
				.withPosition(0, 0).withSize(11, 5);
		} catch (VideoException e) {
			new Alert("Could not connect to USB Camera", AlertType.Error).set(true);
		}
		Command shuffleboardAutoOptions = Commands.parallel(
			Commands.waitSeconds(delayEntry.getBoolean(false) ? 2 : 0),
			Commands.runOnce(() -> intake.resetNoteDetection(noteStartEntry.getBoolean(false))));

		// named commands must be registered before any paths are created
		NamedCommands.registerCommand("Options", shuffleboardAutoOptions);
		NamedCommands.registerCommand("XStance", new XStance(swerve));
		NamedCommands.registerCommand("SpeakerAlign", new AprilTagVision(vision, swerve).withTimeout(1));
		NamedCommands.registerCommand("NoteAlign", new GamePieceVision(vision, swerve).withTimeout(1));
		NamedCommands.registerCommand("SpeakerSetting",
			new SetScoringPosition(arm, shooter, ScoringConstants.subwooferPosition));
		NamedCommands.registerCommand("LimelightSetting",
			new SetScoringPosition(arm, shooter, vision::estimateScoringPosition_math));
		NamedCommands.registerCommand("WaitUntilEnd", Commands.idle().until(() -> DriverStation.getMatchTime() <= 5));
		NamedCommands.registerCommand("WaitUntilVeryEnd", Commands.idle().until(() -> DriverStation.getMatchTime() <= 2));
		NamedCommands.registerCommand("WaitIntake",
			IntakeWithArm.withDelay(intake, arm).beforeStarting(() -> intake.resetNoteDetection()));
		NamedCommands.registerCommand("Index", new SetIntakeSpeed(intake, true).withTimeout(.2));
		NamedCommands.registerCommand("ArmDown", new SetArmPosition(arm, 4));
		NamedCommands.registerCommand("ArmCarry", new SetArmPosition(arm, 40));
		NamedCommands.registerCommand("AmpSetting", new SetScoringPosition(arm, shooter, ScoringConstants.ampPosition));
		NamedCommands.registerCommand("SlowForward",
			Commands.startEnd(() -> swerve.drive(new Translation2d(.3, 0), 0, false), () -> swerve.stop(), swerve)
				.withTimeout(1.5));
		NamedCommands.registerCommand("ResetScoring",
			new SetScoringPosition(arm, shooter, ScoringConstants.lowCarryPosition));
		NamedCommands.registerCommand("ResetScoringHigh",
			new SetScoringPosition(arm, shooter, ScoringConstants.carryPosition));
		NamedCommands.registerCommand("StopShooter", new StopShooter(shooter));
		NamedCommands.registerCommand("RaceTimeout", Commands.idle().withTimeout(5));
		NamedCommands.registerCommand("Print", Commands.print("hello!"));
		// create paths
		final List<String> autoNames = AutoBuilder.getAllAutoNames();
		for (final String autoName : autoNames) {
			if (autoName == "pits test" && !Constants.tuningMode)
				continue;
			final Command auto = new PathPlannerAuto(autoName);
			autoChooser.addOption(autoName, auto);
		}

		if (Constants.tuningMode) {
			tuningModeAlert.set(true);
			Logger.recordOutput("SysId/testState", SysIdRoutineLog.State.kNone.toString()); // populate default

			autoChooser.addOption("sysid swerve drive", sysIdTestSet(swerve.driveSysIdRoutine));
			autoChooser.addOption("sysid swerve angle", sysIdTestSet(swerve.angleSysIdRoutine));
			autoChooser.addOption("sysid shooter", sysIdTestSet(shooter.sysIdRoutine));

			vision.setPriorityId(7);
		}

		// shouldn't require tuningMode as we might run it randomly in comp
		// needs to run when disabled so motors don't run
		SmartDashboard.putData("swerve angle calibration", new SwerveAngleOffsetCalibration(swerve));
		SmartDashboard.putData("Clean Motor", new SetShooterVelocity(shooter, 1500));

		SmartDashboard.updateValues();
		Shuffleboard.update();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// *DRIVER CONTROLS*
		//
		// swerve controls
		var teleopSwerve = new TeleopSwerve(
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX(),
			() -> -driverController.getRightX(),
			() -> false, // no overdrive functionality
			// driverController.getHID()::getLeftBumper, // override speed
			() -> driverController.getLeftTriggerAxis() > OperatorConstants.boolTriggerThreshold, // precise mode
			driverController.getHID()::getLeftStickButton, // super precise mode
			swerve);

		swerve.setDefaultCommand(teleopSwerve);
		driverController.rightTrigger(OperatorConstants.boolTriggerThreshold)
			.whileTrue(teleopSwerve.holdToggleFieldRelative());
		driverController.y().onTrue(teleopSwerve.zeroYaw());
		// right bumper -> limelight arm + shooter setting
		driverController.a().onTrue(teleopSwerve.toggleFieldRelative());

		// limelight shooting
		driverController.rightBumper()
			.onTrue(SetScoringPosition.withVision(arm, shooter, vision));
		// the double colon calls fora method but doesn't return its value immediately
		driverController.leftBumper().whileTrue(new AprilTagVision(vision, swerve));

		// passing just in case
		// driverController.x().whileTrue(new XStance(swerve));
		driverController.x().onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.passPosition));
		driverController.b().whileTrue(new SetIntakeSpeed(intake, true));

		// dpad left -> call for coopertition (lights)
		// dpad right -> call for amplify (lights)

		// *OPERATOR CONTROLS*
		//
		// Scoring presets
		operatorController.a()
			.onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.ampPosition));
		operatorController.b()
			.onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.subwooferPosition));
		operatorController.x().onTrue(
			new SetScoringPosition(arm, shooter, ScoringConstants.carryPosition).andThen(() -> intake.resetNoteDetection()));
		operatorController.y()
			.onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.passPosition));
		operatorController.povLeft().onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.lowPassPosition));

		// arm control
		operatorController.leftBumper().onTrue(new SetArmPosition(arm, 4));
		operatorController.rightBumper().onTrue(new SetArmPosition(arm, 40));
		operatorController.rightStick().whileTrue(new ManualArmControl(arm, operatorController::getLeftY));

		// intaking/indexing
		operatorController.leftTrigger(boolTriggerThreshold)
			.whileTrue(IntakeWithArm.withDelay(intake, arm));
		operatorController.rightTrigger(boolTriggerThreshold).whileTrue(new SetIntakeSpeed(intake, true));
		operatorController.leftStick().whileTrue(new SetIntakeSpeed(intake, -IntakeConstants.intakeSpeed, true));
		operatorController.povRight().onTrue(Commands.runOnce(() -> intake.resetNoteDetection()));

		// only let climb be run if end of match or we aren't on the field
		BooleanSupplier climbAllowed = () -> (DriverStation.getMatchTime() < 32) || (!DriverStation.isFMSAttached());
		operatorController.povDown().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kForward), arm)
			.onlyIf(climbAllowed));
		operatorController.povUp().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kReverse), arm)
			.onlyIf(climbAllowed));
		operatorController.start().toggleOnTrue(new AutoClimb(arm));
		// operatorController.povLeft().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kOff), arm));

		// *RESET YAW THROUGH PUSHBUTTON*
		Trigger yawTrigger = new Trigger(swerve::getYawSwitch);
		yawTrigger.onTrue(teleopSwerve.zeroYaw());

		// *TESTING CONTROLS*
		//
		// When in tuning mode, create multiple testing options on shuffleboard as well as bind commands to a unique
		// 'testing' controller
		if (Constants.tuningMode)

		{
			var indexSpeedEntry = Shuffleboard.getTab("Test").add("Index Speed", 0)
				.withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("min", -1, "max", 1)).getEntry();
			var intakeSpeedEntry = Shuffleboard.getTab("Test").add("Intake Speed", 0)
				.withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("min", -1, "max", 1)).getEntry();
			var shooterSpeedEntry = Shuffleboard.getTab("Test").add("Shooter Speed", 0)
				.withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("min", -1, "max", 1)).getEntry();
			var shooterVelocityEntry = Shuffleboard.getTab("Test").add("Shooter Velocity", 0).getEntry();
			var armPositionEntry = Shuffleboard.getTab("Test").add("Arm Position (degrees)", 0).getEntry();

			testController.x().onTrue(new StopShooter(shooter));
			testController.y()
				.onTrue(Commands.runOnce(() -> arm.setPosition(armPositionEntry.getDouble(boolTriggerThreshold)), arm));
			testController.b()
				.onTrue(new InstantCommand(() -> shooter.setVelocity(shooterVelocityEntry.getDouble(0)), shooter));
			testController.a()
				.onTrue(new InstantCommand(() -> shooter.setPercentOutput(shooterSpeedEntry.getDouble(0)), shooter));

			testController.leftTrigger(boolTriggerThreshold).whileTrue(IntakeWithArm.withDelay(intake, arm));
			testController.leftBumper()
				.whileTrue(Commands
					.runEnd(() -> intake.setMotorSpeed(intakeSpeedEntry.getDouble(0)), () -> intake.stopMotor(), intake)
					.until(() -> intake.inputs.noteSensor).unless(() -> intake.inputs.noteSensor));
			testController.rightTrigger(boolTriggerThreshold).whileTrue(
				new StartEndCommand(() -> intake.setMotorSpeed(indexSpeedEntry.getDouble(0)), () -> intake.stopMotor(),
					intake));

			testController.leftStick().whileTrue(new SetIntakeSpeed(intake, -IntakeConstants.intakeSpeed, true));
			testController.rightStick().whileTrue(new ManualArmControl(arm, testController::getLeftY));

			testController.povDown().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kForward), arm));
			testController.povUp().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kReverse), arm));
			testController.povLeft().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kOff), arm));
		}

		// DEMO CONTROLS
		var demoTab = Shuffleboard.getTab("Demo");

		var demoEnabledEntry = demoTab.add("Demo Mode Enabled?", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
		var demoDriveEnabledEntry = demoTab.add("Demo Drive Enabled?", false).withWidget(BuiltInWidgets.kToggleSwitch)
			.getEntry();

		BooleanSupplier isDemoEnabled = () -> demoEnabledEntry.getBoolean(false);

		var demoDriveScaleEntry = demoTab.add("Drive Scaling", .8).withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)).getEntry();
		var demoTurnScaleEntry = demoTab.add("Turn Scaling", .8).withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)).getEntry();

		var demoAngleEntry = demoTab.add("Arm Angle (deg)", -1.5).getEntry();
		var demoSpeedEntry = demoTab.add("Shooter speed (rpm)", 2700).getEntry();

		var demoSwerve = new TeleopSwerve(
			() -> (-demoController.getLeftY() * demoDriveScaleEntry.getDouble(.8)),
			() -> (-demoController.getLeftX() * demoDriveScaleEntry.getDouble(.8)),
			() -> (-demoController.getRightX() * demoTurnScaleEntry.getDouble(.8)),
			() -> false,
			() -> false, // no alt modes, just one speed scale
			() -> false,
			swerve);

		Trigger demoDriving = new Trigger(
			() -> (demoDriveEnabledEntry.getBoolean(false) && demoEnabledEntry.getBoolean(false)));
		demoDriving.whileTrue(demoSwerve);

		demoController.leftTrigger(boolTriggerThreshold)
			.whileTrue(IntakeWithArm.withDelay(intake, arm).onlyWhile(isDemoEnabled));
		demoController.rightTrigger(boolTriggerThreshold)
			.whileTrue(new SetIntakeSpeed(intake, true).onlyWhile(isDemoEnabled));

		demoController.b().onTrue(new SetScoringPosition(arm, shooter,
			() -> new ScoringPosition(demoAngleEntry.getDouble(-1.5), demoSpeedEntry.getDouble(2700))));
		demoController.x().onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.carryPosition));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.get();
	}

	private Command sysIdTestSet(SysIdRoutine routine) {
		return Commands.sequence(
			routine.quasistatic(SysIdRoutine.Direction.kForward),
			Commands.waitSeconds(3),
			routine.quasistatic(SysIdRoutine.Direction.kReverse),
			Commands.waitSeconds(3),
			routine.dynamic(SysIdRoutine.Direction.kForward),
			Commands.waitSeconds(3),
			routine.dynamic(SysIdRoutine.Direction.kReverse));
	}

	Alert tuningModeAlert = new Alert("tuning mode enabled, expect decreased performance", AlertType.Info);
	Alert driverControllerAlert = new Alert("driver controller not connected properly", AlertType.Error);

	public void updateOIAlert() {
		driverControllerAlert.set(!driverController.getHID().isConnected()
			|| driverController.getHID().getName().indexOf("Xbox") < 0);
	}

	private boolean cachedShooterAtTarget = false;
	private boolean cachedHoldingNote = false;

	/**
	 * Runs checks on the shooter and intake to send rumble to operator/driver controllers.
	 */
	public void checkRumble() {
		// operator rumble on shooter reaching speed
		boolean currShooterAtTarget = shooter.isAtTargetVelocity();
		if (currShooterAtTarget && (currShooterAtTarget != cachedShooterAtTarget)) {
			if (shooter.inputs.leadVelocity_rpm > 400) {
				ControllerUtil.pulseRumble(operatorController).schedule();
			}
		}
		cachedShooterAtTarget = currShooterAtTarget;

		// driver rumble on holding note
		boolean currHoldingNote = intake.isHoldingNote();
		if (currHoldingNote && (currHoldingNote != cachedHoldingNote)) {
			ControllerUtil.pulseRumble(driverController).schedule();
		}
		cachedHoldingNote = currHoldingNote;
	}
}