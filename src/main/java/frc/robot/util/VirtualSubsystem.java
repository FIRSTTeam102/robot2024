package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/** A base for subsystems that do not have mechanisms and command locks */
public abstract class VirtualSubsystem {
	private static List<VirtualSubsystem> subsystems = new ArrayList<>();

	/** Calls {@link #periodic()} on all virtual subsystems */
	public static void periodicAll() {
		for (final var subsystem : subsystems) {
			subsystem.periodic();
		}
	}

	public VirtualSubsystem() {
		subsystems.add(this);
	}

	/** This method is called periodically */
	public abstract void periodic();
}
