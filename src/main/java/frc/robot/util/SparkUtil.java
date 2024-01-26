package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class SparkUtil {
	public static final int framePeriodDisabled = 65535;
	public static final int framePeriodSlow = 500;
	public static final int framePeriodMid = 40;
	public static final int framePeriodDefault = 15;
	public static final int framePeriodFast = 15;

	// https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
	public static void setPeriodicFrames(CANSparkBase spark, boolean hasFollower,
		boolean needVelocity, boolean needPosition,
		boolean analogSensor, boolean alternateEncoder,
		boolean dutyCycleAbsolutePosition, boolean dutyCycleAbsoluteVelocity) {
		// applied output, faults, important information for followers
		spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, hasFollower ? 10 : framePeriodDefault);

		// velocity, temperature, voltage, current
		spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, needVelocity ? framePeriodDefault : framePeriodMid);

		// position
		spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, needPosition ? framePeriodDefault : framePeriodDisabled);

		// opt-in sensors
		spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, analogSensor ? framePeriodDefault : framePeriodDisabled);
		spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, alternateEncoder ? framePeriodDefault : framePeriodDisabled);
		spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5,
			dutyCycleAbsolutePosition ? framePeriodDefault : framePeriodDisabled);
		spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6,
			dutyCycleAbsoluteVelocity ? framePeriodDefault : framePeriodDisabled);
	}

	// https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/CleanSparkMaxValue.java
	public static double cleanValue(double lastValue, double value) {
		if (Double.isNaN(value)
			|| Double.isInfinite(value)) {
			// || (Math.abs(value) < 1.0e-4 && Math.abs(lastValue) > 60.0)) {
			return lastValue;
		} else

		{
			return value;
		}
	}
}
