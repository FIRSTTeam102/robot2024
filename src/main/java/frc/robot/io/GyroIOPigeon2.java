package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 gyro;
	private Pigeon2Configuration config;
	private final StatusSignal<Double> yawSignal;
	private final StatusSignal<Double> pitchSignal;
	private final StatusSignal<Double> rollSignal;
	private final StatusSignal<Double> temperatureSignal;

	public GyroIOPigeon2(int deviceNumber) {
		gyro = new Pigeon2(deviceNumber);
		config = new Pigeon2Configuration()
			.withMountPose(new MountPoseConfigs()
				.withMountPoseYaw(0));
		gyro.getConfigurator().apply(config);
		yawSignal = gyro.getYaw();
		pitchSignal = gyro.getPitch();
		rollSignal = gyro.getRoll();
		temperatureSignal = gyro.getTemperature();
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = true; // fixme:
		inputs.yaw_deg = yawSignal.getValueAsDouble();
		// inputs.yaw_dps = xyz_dps[0];
		inputs.pitch_rad = Units.degreesToRadians(pitchSignal.getValueAsDouble());
		// inputs.pitch_radps = Units.degreesToRadians(xyz_dps[1]);
		inputs.roll_rad = Units.degreesToRadians(rollSignal.getValueAsDouble());
		// inputs.roll_radps = Units.degreesToRadians(xyz_dps[2]);
		inputs.temperature_C = temperatureSignal.getValueAsDouble();
	}

	@Override
	public void setYaw(double yaw_deg) {
		gyro.setYaw(yaw_deg);
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
