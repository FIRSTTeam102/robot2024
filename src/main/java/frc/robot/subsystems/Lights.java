
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import frc.robot.constants.LightsConstants.Mode;

import edu.wpi.first.wpilibj.DigitalOutput;

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

	/**
	 * Sends the desired mode to the lights arduino through the DIO pins
	 * @param mode Desired lights mode
	 */
	public static void setStatus(Mode mode) {
		int code = mode.getCode();
		BitSet bits = BitSet.valueOf(new byte[] {(byte) code});
		for (int i = 0; i <= 3; i++) {
			pins[i].set(bits.get(i));
		}

		Logger.recordOutput("Lights/CurrentStatus", mode);
	}
}
