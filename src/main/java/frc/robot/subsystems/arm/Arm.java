package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
//import frc.robot.util.ArmVisualizer3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
	private final ArmIO io;
	// private final ArmIOInputsAutoLogged inputs;
	// private final ArmVisualizer3d armVisualizer;

	public Arm(ArmIO io) {
		this.io = io;
		// this.inputs = new ArmIOInputsAutoLogged();
		// //this.armVisualizer = new ArmVisualizer3d(getName(), new Translation3d(0,0.378-0.044,0.184), Rotation2d.fromDegrees(0));
		// this.armVisualizer = new ArmVisualizer3d(getName(), new Translation3d(0,0.333375,0.196815), Rotation2d.fromDegrees(0));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Arm", inputs);

	// 	armVisualizer.setArmAngle(inputs.pivotPosition);
	// 	armVisualizer.publish();
	}

	public Command setWheelVoltage(Supplier<Voltage> voltage) {
		return Commands.runOnce(() -> io.setWheelVoltage(voltage.get()), this);
	}


	public Command setWheelVelocity(Supplier<AngularVelocity> velocity) {
		return Commands.runOnce(() -> io.setWheelVelocitySetpoint(velocity.get()), this);
	}

	public boolean atPosition() {
		return inputs.pivotPosition.minus(inputs.pivotDesiredPosition).abs(Degrees) < 2;
	}

	public Command setPivotPosition(Supplier<Angle> position) {
		return Commands.runOnce(() -> io.setPivotPosition(position.get()), this);
	}

	public Command setPivotPositionSetpoint(Supplier<Angle> position) {
		return Commands.runOnce(() -> io.setPivotPositionSetpoint(position.get()), this);
	}

	public Command setPivotVoltage(Supplier<Voltage> volts) {
		return Commands.runOnce(() -> io.setPivotVoltage(volts.get()), this);
	}

	public Command setPivotVoltageDirect(Supplier<Voltage> volts) {
		return Commands.runOnce(() -> io.setPivotVoltageDirect(volts.get()), this);
	}

	public Angle getPivotPosition() {
		return inputs.pivotPosition;
	}

	public Command sysIdRoutinePivot() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.per(Second).of(0.25),
				Volts.of(1),
				null,
				(state) -> Logger.recordOutput(
					"SysId/Arm-pivot", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setPivotVoltage,
				log -> {
					Logger.recordOutput("SysId/Arm-pivot/Voltage", inputs.pivotAppliedVolts);
					Logger.recordOutput("SysId/Arm-pivot/Position", inputs.pivotPosition);
					Logger.recordOutput("SysId/Arm-pivot/Velocity", inputs.pivotVelocity);
					log.motor("Arm-pivot")
						.voltage(inputs.pivotAppliedVolts)
						.angularPosition(inputs.pivotPosition)
						.angularVelocity(inputs.pivotVelocity);
				}, 
				this)
		);


		Command routineCommand = new SequentialCommandGroup(
			routine.dynamic(Direction.kReverse).until(() -> inputs.pivotPosition.lte(Degrees.of(20))),
			Commands.waitSeconds(5),
			routine.dynamic(Direction.kForward).until(() -> inputs.pivotPosition.gte(Degrees.of(90))),
			Commands.waitSeconds(5),
			routine.quasistatic(Direction.kReverse).until(() -> inputs.pivotPosition.lte(Degrees.of(20))),
			Commands.waitSeconds(5),
			routine.quasistatic(Direction.kForward).until(() -> inputs.pivotPosition.gte(Degrees.of(90)))
		);
		

		return routineCommand;
	}
	public Command sysIdRoutineWheel() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(0.2).per(Second),
				Volts.of(1),
				null,
				(state) -> Logger.recordOutput(
					"SysId/Arm-wheel", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setWheelVoltage,
				log -> {
					Logger.recordOutput("SysId/Arm-wheel/Voltage", inputs.wheelAppliedVolts);
					Logger.recordOutput("SysId/Arm-wheel/Velocity", inputs.wheelVelocity);
					Logger.recordOutput("SysId/Arm-wheel/Position", inputs.wheelPosition);
					log.motor("Arm-wheel")
						.voltage(inputs.wheelAppliedVolts)
						.angularPosition(inputs.wheelPosition)
						.angularVelocity(inputs.wheelVelocity);
				}, 
				this)
		);


		Command routineCommand = new SequentialCommandGroup(
			routine.dynamic(Direction.kReverse),
			Commands.waitSeconds(1),
			routine.dynamic(Direction.kForward),
			Commands.waitSeconds(1),
			routine.quasistatic(Direction.kReverse),
			Commands.waitSeconds(1),
			routine.quasistatic(Direction.kForward)
		);
		return routineCommand;
	}

	public void enabledInit() {
		this.io.enabledInit();
	}

}