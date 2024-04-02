// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import lombok.Getter;

public final class LightsConstants {
	public enum Mode {
		Shooting(0), Intaking(1), Disabled(2), Auto(3), CanShoot(4), TeleopRED(5), TeleopBLUE(6), Climb(
			7), HaveANote(8);

		@Getter
		private final int code;

		private Mode(int code) {
			this.code = code;
		}
	}
}