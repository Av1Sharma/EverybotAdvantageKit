package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class ArmConstants {
	public static final int Arm_MOTOR_ID = 15;
	public static final int PIVOT_MOTOR_ID = 16;

	public static final Angle STARTING_ANGLE = Degrees.of(96.0);

	public static final double WHEEL_CONVERSION_FACTOR = 2 * Math.PI;
	public static final double PIVOT_CONVERSION_FACTOR = 2 * Math.PI * (1.0/25.0) * (9.0/12.0); // 16:1 gear ratio, 18/12 sprocket ratio

	public static final double PIVOT_kP = 10;
	public static final double PIVOT_kI = 0;
	public static final double PIVOT_kD = 0;
	public static final double PIVOT_SIM_kP = 88.636;
	public static final double PIVOT_SIM_kI = 0;
	public static final double PIVOT_SIM_kD = 6.458;
	public static final double PIVOT_kS = 0.36769;
	public static final double PIVOT_kG = 0.25887;
	public static final double PIVOT_kV = 0.56944;
	public static final double PIVOT_kA = 0.058227;
	public static final double PIVOT_SIM_kS = 1.024;
	public static final double PIVOT_SIM_kG = 7.7827;
	public static final double PIVOT_SIM_kV = 0.069055;
	public static final double PIVOT_SIM_kA = 0.24078;

	public static final double WHEEL_kP = 0.001;
	public static final double WHEEL_kI = 0;
	public static final double WHEEL_kD = 0;
	public static final double WHEEL_SIM_kP = 3.596;
	public static final double WHEEL_SIM_kI = 0;
	public static final double WHEEL_SIM_kD = 0;
	public static final double WHEEL_kS = 0.15912;
	public static final double WHEEL_kV = 0.021763;
	public static final double WHEEL_kA = 0.0014371;
	public static final double WHEEL_SIM_kS = 6.6332;
	public static final double WHEEL_SIM_kV = 1688.8;
	public static final double WHEEL_SIM_kA = 0;

	public static final Mass WHEEL_MASS = Pounds.of(0.035);
	public static final Distance WHEEL_RADIUS = Inches.of(1);
	public static final double WHEEL_MOI = 0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2); // 1/2MR^2

	public static final Distance LENGTH = Inches.of(18);
	public static final Mass MASS = Pounds.of(5.2);
	public static final double PIVOT_ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;
}