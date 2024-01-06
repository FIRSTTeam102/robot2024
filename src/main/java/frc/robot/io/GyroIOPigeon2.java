package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 gyro;
	private Pigeon2Configuration config;
	private final StatusSignal<Double> yawSignal;
	private final StatusSignal<Double> angularVelocityZSignal;
	private final StatusSignal<Double> pitchSignal;
	private final StatusSignal<Double> angularVelocityYSignal;
	private final StatusSignal<Double> rollSignal;
	private final StatusSignal<Double> angularVelocityXSignal;
	private final StatusSignal<Double> temperatureSignal;

	private final int importantUpdateFrequency_Hz = 100;
	private final int unimportantUpdateFrequency_Hz = 50;

	public GyroIOPigeon2(int deviceNumber) {
		gyro = new Pigeon2(deviceNumber);
		config = new Pigeon2Configuration()
			.withMountPose(new MountPoseConfigs()
				.withMountPoseYaw(0));
		gyro.getConfigurator().apply(config);
		yawSignal = gyro.getYaw();
		yawSignal.setUpdateFrequency(importantUpdateFrequency_Hz);
		angularVelocityZSignal = gyro.getAngularVelocityZWorld();
		angularVelocityZSignal.setUpdateFrequency(importantUpdateFrequency_Hz);
		pitchSignal = gyro.getPitch();
		pitchSignal.setUpdateFrequency(unimportantUpdateFrequency_Hz);
		angularVelocityYSignal = gyro.getAngularVelocityYWorld();
		angularVelocityYSignal.setUpdateFrequency(unimportantUpdateFrequency_Hz);
		rollSignal = gyro.getRoll();
		rollSignal.setUpdateFrequency(unimportantUpdateFrequency_Hz);
		angularVelocityXSignal = gyro.getAngularVelocityXWorld();
		angularVelocityXSignal.setUpdateFrequency(unimportantUpdateFrequency_Hz);
		temperatureSignal = gyro.getTemperature();
		temperatureSignal.setUpdateFrequency(unimportantUpdateFrequency_Hz);
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		// https://github.com/HuskieRobotics/3061-lib/blob/main/src/main/java/frc/lib/team3061/gyro/GyroIOPigeon2Phoenix6.java

		BaseStatusSignal.refreshAll(yawSignal, pitchSignal, rollSignal, temperatureSignal);

		inputs.connected = yawSignal.getStatus() == StatusCode.OK;

		inputs.yaw_deg = BaseStatusSignal.getLatencyCompensatedValue(yawSignal, angularVelocityZSignal);
		// inputs.yaw_dps = xyz_dps[0];
		inputs.pitch_rad = Units
			.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(pitchSignal, angularVelocityYSignal));
		// inputs.pitch_radps = Units.degreesToRadians(xyz_dps[1]);
		inputs.roll_rad = Units
			.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(rollSignal, angularVelocityXSignal));
		// inputs.roll_radps = Units.degreesToRadians(xyz_dps[2]);
		inputs.temperature_C = temperatureSignal.getValueAsDouble();
	}

	@Override
	public void setYaw(double yaw_deg) {
		gyro.setYaw(yaw_deg, 0.1);
	}

	@Override
	public void resetPitchRoll() {
		config = config
			.withMountPose(new MountPoseConfigs()
				.withMountPosePitch(pitchSignal.getValueAsDouble())
				.withMountPoseRoll(rollSignal.getValueAsDouble()));
		gyro.getConfigurator().apply(config);
	}
}
