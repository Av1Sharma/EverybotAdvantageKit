package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface ArmIO {
	@AutoLog
	public static class ArmIOInputs {
		public Angle wheelPosition = Radians.of(0);
		public AngularVelocity wheelVelocity = RadiansPerSecond.of(0);
		public AngularVelocity wheelDesiredVelocity = RadiansPerSecond.of(0);
		public AngularVelocity wheelVelocitySetpoint = RadiansPerSecond.of(0);
		public Voltage wheelAppliedVolts = Volts.of(0);
		public Current wheelSupplyCurrent = Amps.of(0);
		public Temperature wheelMotorTemperature = Celsius.of(0);

		public Angle pivotDesiredPositionSetpoint = Radians.of(0);
		public Angle pivotDesiredPosition = Radians.of(0);
		public AngularVelocity pivotVelocity = RadiansPerSecond.of(0);
		public Angle pivotPosition = Radians.of(0);
		public Voltage pivotAppliedVolts = Volts.of(0);
		public Current pivotSupplyCurrent = Amps.of(0);
		public Temperature pivotTemperature = Celsius.of(0);
	}

	public default void updateInputs(ArmIOInputs inputs) {
	}

	public default void stopWheel() {
	}

	public default void stopPivot() {
	}

	public default void setWheelVelocitySetpoint(AngularVelocity velocity) {
	}

	public default void setWheelVoltage(Voltage volts) {
	}

	public default void setPivotPosition(Angle position) {
	}

	public default void setPivotPositionSetpoint(Angle position) {
	}

	public default void setPivotVoltage(Voltage volts) {
	}

	public default void setPivotVoltageDirect(Voltage volts) {
	}

	public default void enabledInit() {}

}