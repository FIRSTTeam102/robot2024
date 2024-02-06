package frc.robot;

import static frc.robot.constants.Constants.OperatorConstants.*;

import static frc.robot.constants.ShooterConstants.shooterVelocity;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.ShuffleboardConstants;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SystemAlerter;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;


import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.Shooter.SetShooterVelocity;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.swerve.SwerveAngleOffsetCalibration;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("auto routine");

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/* subsystems */
	// public final Vision vision = new Vision();
	public final Arm arm = new Arm();
	public final Intake intake = new Intake();
	public final Swerve swerve = new Swerve(gyro/* , vision */);
	public final Shooter shooter = new Shooter();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		new SystemAlerter();

		configureBindings();

		// named commands must be registered before any paths are created
		NamedCommands.registerCommand("XStance", new XStance(swerve));
		NamedCommands.registerCommand("AimAndShoot",
			Commands.print("aiming and shooting").andThen(Commands.waitSeconds(1)));
		NamedCommands.registerCommand("Intake", Commands.print("arm to intaking position & rollers running"));
		NamedCommands.registerCommand("WaitIntake",
			Commands.print("wait for intake note sensor").andThen(Commands.waitSeconds(1)));
		NamedCommands.registerCommand("SafeArm", Commands.print("move arm to safe position inside frame"));

		// create paths
		autoChooser.addOption("nothing", Commands.none());
		final List<String> autoNames = AutoBuilder.getAllAutoNames();
		for (final String autoName : autoNames) {
			final Command auto = new PathPlannerAuto(autoName);
			autoChooser.addOption(autoName, auto);
		}

		final var driveTab = Shuffleboard.getTab(ShuffleboardConstants.driveTab);
		driveTab.add("auto routine", autoChooser.getSendableChooser())
			.withSize(4, 1).withPosition(0, 5);
		driveTab.add("alerts", Alert.getAlertsSendable())
			.withSize(5, 4).withPosition(4, 5).withWidget(Alert.widgetName);
		driveTab.add("camera", SendableCameraWrapper.wrap("limelight-stream", "http://10.1.2.11:5800/stream.mjpg"))
			.withProperties(Map.of("show crosshair", false, "show controls", false))
			.withWidget(BuiltInWidgets.kCameraStream)
			.withSize(11, 5)
			.withPosition(0, 0);

		if (Constants.tuningMode) {
			tuningModeAlert.set(true);
			Logger.recordOutput("SysId/testState", SysIdRoutineLog.State.kNone.toString()); // populate default

			autoChooser.addOption("sysid swerve drive", sysIdTestSet(swerve.driveSysIdRoutine));
			autoChooser.addOption("sysid swerve angle", sysIdTestSet(swerve.angleSysIdRoutine));
			autoChooser.addOption("sysid shooter", sysIdTestSet(shooter.sysIdRoutine));
		}

		// shouldn't require tuningMode as we might run it randomly in comp
		// needs to run when disabled so motors don't run
		SmartDashboard.putData("swerve angle calibration", new SwerveAngleOffsetCalibration(swerve));
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
		var teleopSwerve = new TeleopSwerve(
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX(),
			() -> -driverController.getRightX(),
			driverController.getHID()::getLeftBumper, // override speed
			() -> driverController.getLeftTriggerAxis() > OperatorConstants.boolTriggerThreshold, // preceise mode
			swerve);
		swerve.setDefaultCommand(teleopSwerve);

		driverController.rightTrigger(OperatorConstants.boolTriggerThreshold)
			.whileTrue(teleopSwerve.holdToggleFieldRelative());
		// driverController.rightBumper()
		// .whileTrue(teleopSwerve.holdRotateAroundPiece());

		driverController.a().onTrue(teleopSwerve.toggleFieldRelative());
		driverController.x().whileTrue(new XStance(swerve));
		driverController.y().onTrue(teleopSwerve.zeroYaw());


    operatorController.y().onTrue(new SetShooterVelocity(shooter, shooterVelocity));
		operatorController.x().onTrue(new StopShooter(shooter));
		operatorController.b().onTrue(new InstantCommand(() -> shooter.setPercentOutput(.85), shooter));
    operatorController.rightBumper().whileTrue(new SetIntakeSpeed(intake));
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
			Commands.waitSeconds(2),
			routine.quasistatic(SysIdRoutine.Direction.kReverse),
			Commands.waitSeconds(2),
			routine.dynamic(SysIdRoutine.Direction.kForward),
			Commands.waitSeconds(2),
			routine.dynamic(SysIdRoutine.Direction.kReverse));
	}

	Alert tuningModeAlert = new Alert("tuning mode enabled, expect decreased performance", AlertType.Info);
	Alert driverControllerAlert = new Alert("driver controller not connected properly", AlertType.Error);

	public void updateOIAlert() {
		driverControllerAlert.set(!driverController.getHID().isConnected()
			|| driverController.getHID().getName().indexOf("Xbox") < 0);
	}
}
