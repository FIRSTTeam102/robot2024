package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

public class AutoSetterTunableNumber extends TunableNumber {
	public static class AutoSetterTunableNumberManager {
		private static List<AutoSetterTunableNumber> instances = new ArrayList<>();

		public static void addInstance(AutoSetterTunableNumber instance) {
			instances.add(instance);
		}

		public static void periodic() {
			for (final var instance : instances) {
				instance.periodic();
			}
		}
	}

	private DoubleConsumer valueSettingFunction;

	public AutoSetterTunableNumber(String dashboardKey, DoubleConsumer valueSettingFunction) {
		super(dashboardKey);
		this.valueSettingFunction = valueSettingFunction;
		AutoSetterTunableNumberManager.addInstance(this);
	}

	public AutoSetterTunableNumber(String dashboardKey, double defaultValue, DoubleConsumer valueSettingFunction) {
		super(dashboardKey, defaultValue);
		this.valueSettingFunction = valueSettingFunction;
		AutoSetterTunableNumberManager.addInstance(this);
	}

	/** note: this will only do something when in tuning mode */
	private void periodic() {
		if (hasChanged()) {
			valueSettingFunction.accept(lastHasChangedValue);
		}
	}
}
