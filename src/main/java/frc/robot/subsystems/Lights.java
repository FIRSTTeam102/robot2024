// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.LightsConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
	private DigitalInput firstlightInput = new DigitalInput(LightsConstants.firstLightInputPin);
	private DigitalInput secondlightInput = new DigitalInput(LightsConstants.secondLightInputPin);
	private DigitalInput threeDigitalInput = new DigitalInput(LightsConstants.thirdLightInputPin);

	public static boolean[] lightArray = new boolean[8]; // The array that will be used for lights

	public Lights() {}

	@Override
	public void periodic() { // optimise to escape as soon as it hits a true
		if (Lights.lightArray[LightsConstants.orderAmplify] == true) {
			// send arduino byte
		}
		if (Lights.lightArray[LightsConstants.orderCoop] == true) {
			// send arduino byte
		}
		if (Lights.lightArray[LightsConstants.orderIntake] == true) {
			// send arduino byte
		}
		if (Lights.lightArray[LightsConstants.shooterSpunUp] == true) {
			// send arduino byte
		}
		if (Lights.lightArray[LightsConstants.noteClaimed] == true) {
			// send arduino byte
		}
		if (Lights.lightArray[LightsConstants.noteShot] == true) {
			// send arduino byte
		}

	}

	public void sendByte(boolean firstDigit, boolean secondDigit, boolean thirdDigit) {
		firstlightInput = firstDigit; // why doesn't this work
	}

	public void callCoop(boolean called) {
		if (called) { // no need to check since it's a boolean. It's automatically assumed true
			Lights.lightArray[LightsConstants.orderCoop] = true;
		} else {
			Lights.lightArray[LightsConstants.orderCoop] = false;
		}
	}

	public void callAmplify(boolean called) {
		if (called) { // no need to check since it's a boolean. It's automatically assumed true
			Lights.lightArray[LightsConstants.orderAmplify] = true;
		} else {
			Lights.lightArray[LightsConstants.orderAmplify] = false;
		}
	}
}
