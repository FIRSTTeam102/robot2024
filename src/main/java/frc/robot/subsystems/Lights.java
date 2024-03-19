
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import frc.robot.constants.Constants;
import frc.robot.constants.LightsConstants.Mode;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;

import java.util.BitSet;

public class Lights {
	private static DigitalOutput[] pins = {
		new DigitalOutput(1),
		new DigitalOutput(2),
		new DigitalOutput(3),
		new DigitalOutput(4)
	};

	public Lights() {}

	public static void setStatus(Mode mode) {
		int code = mode.getCode();
		BitSet bits = BitSet.valueOf(new byte[] {(byte) code});
		for (int i = 0; i <= 3; i++) {
			if (Constants.tuningMode)
				Logger.recordOutput("Lights/bits_" + i, bits.get(i) ? 1 : 0);
			pins[i].set(bits.get(i));
		}

		Logger.recordOutput("Lights/CurrentStatus", mode);
	}

	public static void setDefaultStatus() {
		if (DriverStation.isDisabled()) {
			setStatus(Mode.Disabled);
			return;
		}
		if (DriverStation.isAutonomous()) {
			setStatus(Mode.Auto);
			return;
		}

		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			switch (alliance.get()) {
				case Red -> setStatus(Mode.TeleopRED);
				case Blue -> setStatus(Mode.TeleopBLUE);
			}
		}
	}
}
