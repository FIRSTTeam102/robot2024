package frc.robot.subsystems;

import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.VirtualSubsystem;

import edu.wpi.first.math.filter.LinearFilter;

import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.inputs.LoggedSystemStats.SystemStatsInputs;

public class SystemAlerter extends VirtualSubsystem {
	private final SystemStatsInputs sysInputs = LoggedSystemStats.getInputs();

	private int batteryWindowSize = 50;
	private LinearFilter batteryVoltageFilter = LinearFilter.movingAverage(batteryWindowSize);

	private final Alert batteryWarningAlert = new Alert("battery getting low, consider replacing", AlertType.Warning);
	private final Alert batteryErrorAlert = new Alert("replace battery", AlertType.Error);

	private final Alert canErrorAlert = new Alert("CAN bus error", AlertType.Error);

	/** Handles alerts for system */
	public SystemAlerter() {
		for (int i = 0; i < batteryWindowSize; i++) {
			batteryVoltageFilter.calculate(sysInputs.voltageVin); // prefill
		}
	}

	@Override
	public void periodic() {
		final double average_V = batteryVoltageFilter.calculate(sysInputs.voltageVin);
		if (average_V < 9 || sysInputs.brownedOut) {
			batteryErrorAlert.set(true);
			// if (DriverStation.getMatchType() == DriverStation.MatchType.None)
			// throw new java.lang.RuntimeException("Replace battery");
		} else if (average_V < 10) {
			batteryWarningAlert.set(true);
		}

		canErrorAlert.set(sysInputs.canStatus.receiveErrorCount > 0 || sysInputs.canStatus.transmitErrorCount > 0);
	}
}
