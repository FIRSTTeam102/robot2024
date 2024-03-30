package frc.robot;

import static frc.robot.constants.Constants.OperatorConstants.*;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.ShuffleboardConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ScoringConstants;
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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
		driveTab.add("camera", SendableCameraWrapper.wrap("limelight-stream", "http://10.1.2.12:5800/stream.mjpg"))
			.withProperties(Map.of("show crosshair", false, "show controls", false))
			.withWidget(BuiltInWidgets.kCameraStream)
			.withSize(11, 5)
			.withPosition(0, 0);

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
			IntakeWithArm.intakeWithDelay(intake, arm).beforeStarting(() -> intake.resetNoteDetection()));
		NamedCommands.registerCommand("Index", new SetIntakeSpeed(intake, true).withTimeout(1));
		NamedCommands.registerCommand("ArmDown", new SetArmPosition(arm, 4));
		NamedCommands.registerCommand("ArmCarry", new SetArmPosition(arm, 40));
		NamedCommands.registerCommand("AmpSetting", new SetScoringPosition(arm, shooter, ScoringConstants.ampPosition));
		NamedCommands.registerCommand("SlowForward",
			Commands.startEnd(() -> swerve.drive(new Translation2d(.3, 0), 0, false), () -> swerve.stop(), swerve)
				.withTimeout(1.5));
		NamedCommands.registerCommand("ResetScoring",
			new SetScoringPosition(arm, shooter, ScoringConstants.lowCarryPosition));
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
		var teleopSwerve = new TeleopSwerve(
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX(),
			() -> -driverController.getRightX(),
			() -> false, // no overdrive functionality
			// driverController.getHID()::getLeftBumper, // override speed
			() -> driverController.getLeftTriggerAxis() > OperatorConstants.boolTriggerThreshold, // precise mode
			driverController.getHID()::getLeftBumper, // super precise mode
			swerve);
		swerve.setDefaultCommand(teleopSwerve);

		driverController.rightTrigger(OperatorConstants.boolTriggerThreshold)
			.whileTrue(teleopSwerve.holdToggleFieldRelative());
		// right bumper -> rotate to speaker apriltag
		driverController.rightBumper().whileTrue(new AprilTagVision(vision, swerve));
		// left bumper -> rotate to note
		// driverController.leftBumper().whileTrue(new GamePieceVision(vision, swerve));
		driverController.a().onTrue(teleopSwerve.toggleFieldRelative());
		driverController.x().whileTrue(new XStance(swerve));
		driverController.y().onTrue(teleopSwerve.zeroYaw());

		// dpad left -> call for coopertition (lights)
		// dpad right -> call for amplify (lights)

		// *OPERATOR CONTROLS*
		//
		operatorController.a()
			.onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.ampPosition));
		operatorController.b()
			.onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.subwooferPosition));
		operatorController.x().onTrue(
			new SetScoringPosition(arm, shooter, ScoringConstants.carryPosition).andThen(() -> intake.resetNoteDetection()));
		operatorController.y()
			.onTrue(new SetScoringPosition(arm, shooter, ScoringConstants.passPosition));
		operatorController.leftBumper().onTrue(new SetArmPosition(arm, 4));
		operatorController.rightBumper().onTrue(new SetArmPosition(arm, 40));
		operatorController.leftTrigger(boolTriggerThreshold)
			.whileTrue(IntakeWithArm.intakeWithDelay(intake, arm));
		operatorController.rightTrigger(boolTriggerThreshold).whileTrue(new SetIntakeSpeed(intake, true));
		operatorController.povDown().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kForward), arm)
			.unless(() -> DriverStation.getMatchTime() > 32));
		operatorController.povUp().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kReverse), arm)
			.unless(() -> DriverStation.getMatchTime() > 32));
		// operatorController.povLeft().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kOff), arm));
		operatorController.povRight().onTrue(Commands.runOnce(() -> intake.resetNoteDetection()));

		operatorController.start().toggleOnTrue(new AutoClimb(arm));

		operatorController.leftStick().whileTrue(new SetIntakeSpeed(intake, -IntakeConstants.intakeSpeed, true));
		operatorController.rightStick().whileTrue(
			new ManualArmControl(arm, operatorController::getLeftY));

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

			testController.leftTrigger(boolTriggerThreshold)
				.whileTrue(Commands
					.runEnd(() -> intake.setMotorSpeed(intakeSpeedEntry.getDouble(0)), () -> intake.stopMotor(), intake)
					.until(() -> intake.inputs.noteSensor).unless(() -> intake.inputs.noteSensor));
			testController.leftBumper().whileTrue(
				new StartEndCommand(() -> intake.setMotorSpeed(indexSpeedEntry.getDouble(0)), () -> intake.stopMotor(),
					intake));

			testController.leftStick().whileTrue(new SetIntakeSpeed(intake, -IntakeConstants.intakeSpeed, true));
			testController.rightStick().whileTrue(new ManualArmControl(arm, testController::getLeftY));

			testController.povDown().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kForward), arm));
			testController.povUp().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kReverse), arm));
			testController.povLeft().onTrue(Commands.runOnce(() -> arm.setClimberRelay(Relay.Value.kOff), arm));
		}
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

	public Command pulseRumble(CommandXboxController controller) {
		return Commands.startEnd(() -> controller.getHID().setRumble(RumbleType.kBothRumble, .65),
			() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0), new Subsystem[] {}).withTimeout(.55);
	}

	private boolean cachedShooterAtTarget = false;
	private boolean cachedHoldingNote = false;

	public void checkRumble() {
		// operator rumble on shooter reaching speed
		boolean currShooterAtTarget = shooter.isAtTargetVelocity();
		if (currShooterAtTarget && (currShooterAtTarget != cachedShooterAtTarget)) {
			if (shooter.inputs.leadVelocity_rpm > 400) {
				pulseRumble(operatorController).schedule();
			}
		}
		cachedShooterAtTarget = currShooterAtTarget;

		// driver rumble on holding note
		boolean currHoldingNote = intake.isHoldingNote();
		if (currHoldingNote && (currHoldingNote != cachedHoldingNote)) {
			pulseRumble(driverController).schedule();
		}
		cachedHoldingNote = currHoldingNote;
	}
}