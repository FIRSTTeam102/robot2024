// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmWave extends SequentialCommandGroup {
	private Arm arm;

	public ArmWave(Arm arm2, double verticalarmposdemoDeg) {
		repeatedly(SetArmPosition(arm, ArmConstants.SetArmPosition(arm, 40))
			.andThen(SetArmPosition(arm, ArmConstants.verticalArmPosDemo_deg)));
		addCommands();
	}
}
