package frc.robot.commands.swerve;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.ControllerUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
	public boolean fieldRelative = true;
	public boolean overrideSpeed = false;

	public Command toggleFieldRelative() {
		return Commands.runOnce(() -> fieldRelative = !fieldRelative);
	};

	public Command holdToggleFieldRelative() {
		return Commands.startEnd(
			() -> fieldRelative = !fieldRelative,
			() -> fieldRelative = !fieldRelative);
	};

	public Command holdRotateAroundPiece() {
		return Commands.startEnd(
			() -> {
				swerve.setCenterRotation(SwerveConstants.trackWidth_m + Units.inchesToMeters(12), 0);
			},
			() -> {
				swerve.setCenterRotation(0, 0);
			});
	}

	private static Alert zeroYawAlert = new Alert("didn't zero yaw", AlertType.Warning);

	private class ZeroYaw extends InstantCommand {
		public ZeroYaw() {
			zeroYawAlert.set(true);
		}

		@Override
		public void initialize() {
			swerve.zeroYaw();
			zeroYawAlert.set(false);
		}

		@Override
		public boolean runsWhenDisabled() {
			return true;
		}
	}

	public Command zeroYaw() {
		return new ZeroYaw();
	}

	private Swerve swerve;
	private DoubleSupplier driveSupplier;
	private DoubleSupplier strafeSupplier;
	private DoubleSupplier turnSupplier;
	private BooleanSupplier overrideSpeedSupplier;
	private BooleanSupplier preciseModeSupplier;
	private BooleanSupplier superPreciseModeSupplier;

	/**
	 * @param overrideSpeedSupplier forces swerve to run at normal speed when held, instead of slow if scoring mechanism is out
	 */
	public TeleopSwerve(DoubleSupplier driveSupplier, DoubleSupplier strafeSupplier, DoubleSupplier turnSupplier,
		BooleanSupplier overrideSpeedSupplier, BooleanSupplier preciseModeSupplier,
		BooleanSupplier superPreciseModeSupplier,
		Swerve swerve) {
		addRequirements(swerve);
		this.driveSupplier = driveSupplier;
		this.strafeSupplier = strafeSupplier;
		this.turnSupplier = turnSupplier;
		this.overrideSpeedSupplier = overrideSpeedSupplier;
		this.preciseModeSupplier = preciseModeSupplier;
		this.superPreciseModeSupplier = superPreciseModeSupplier;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {}

	private double rotation;
	private Translation2d translation;
	private double driveMax = 1.0;
	private double turnMaxPercent = 1.0;

	// todo: tune, see if stopping takes too long and maybe add an override
	private static final double accelerationLimit_mps2 = 13;
	private SlewRateLimiter driveLimiter = new SlewRateLimiter(accelerationLimit_mps2);
	private SlewRateLimiter strafeLimiter = new SlewRateLimiter(accelerationLimit_mps2);

	private static final double normalMaxPercent = 1.0; // todo: depending on driver

	@Override
	public void execute() {
		if (overrideSpeedSupplier.getAsBoolean()) {
			driveMax = 1.0;
			turnMaxPercent = 0.9;
		} else {
			driveMax = normalMaxPercent;
			turnMaxPercent = driveMax * 0.9;
		}

		if (superPreciseModeSupplier.getAsBoolean()) {
			driveMax *= 0.1;
			turnMaxPercent *= 0.15;
		} else if (preciseModeSupplier.getAsBoolean()) {
			driveMax *= 0.3;
			turnMaxPercent *= 0.2;
		}

		driveMax *= SwerveConstants.maxVelocity_mps; // turn percent into velocity

		translation = new Translation2d(
			driveLimiter.calculate(driveMax * ControllerUtil.scaleAxis(driveSupplier.getAsDouble())),
			strafeLimiter.calculate(driveMax * ControllerUtil.scaleAxis(strafeSupplier.getAsDouble())));

		Logger.recordOutput("TeleopSwerve/translationX_mps", translation.getX());
		Logger.recordOutput("TeleopSwerve/translationY_mps", translation.getY());

		rotation = ControllerUtil.scaleAxis(turnSupplier.getAsDouble())
			* SwerveConstants.maxAngularVelocity_radps
			* turnMaxPercent;

		Logger.recordOutput("TeleopSwerve/rotation_radps", rotation);

		swerve.drive(translation, rotation, fieldRelative);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}
}
