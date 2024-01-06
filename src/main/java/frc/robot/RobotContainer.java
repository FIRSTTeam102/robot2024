package frc.robot;

import static frc.robot.constants.Constants.OperatorConstants.driverControllerPort;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.ShuffleboardConstants;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("auto routine");

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/* subsystems */
	// public final Vision vision = new Vision();
	public final Swerve swerve = new Swerve(gyro/* , vision */);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		configureBindings();

		autoChooser.addOption("nothing", Commands.none());
		autoChooser.addDefaultOption("example", new PathPlannerAuto("Example Auto"));

		var driveTab = Shuffleboard.getTab(ShuffleboardConstants.driveTab);
		driveTab.add("auto routine", autoChooser.getSendableChooser())
			.withSize(4, 1).withPosition(0, 5);
		driveTab.add("alerts", Alert.getAlertsSendable())
			.withSize(5, 4).withPosition(4, 5);
		driveTab.add("camera", SendableCameraWrapper.wrap("limelight-stream", "http://10.1.2.11:5800/stream.mjpg"))
			.withProperties(Map.of("show crosshair", false, "show controls", false))
			.withWidget(BuiltInWidgets.kCameraStream)
			.withSize(11, 5)
			.withPosition(0, 0);
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
		driverController.rightBumper()
			.whileTrue(teleopSwerve.holdRotateAroundPiece());

		driverController.a().onTrue(teleopSwerve.toggleFieldRelative());
		driverController.x().whileTrue(new XStance(swerve));
		driverController.y().onTrue(teleopSwerve.zeroYaw());
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

	Alert driverControllerAlert = new Alert("driver controller not connected properly", AlertType.Error);

	public void updateOIAlert() {
		driverControllerAlert.set(!driverController.getHID().isConnected()
			|| driverController.getHID().getName().indexOf("Xbox") < 0);
	}
}
