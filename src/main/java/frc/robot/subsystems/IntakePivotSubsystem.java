package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivotSubsystem extends SubsystemBase {
    private final SmartMotorController smctl;
    private final Pivot intakepivot;

    private final AnalogInput abEncoder;

    public IntakePivotSubsystem() {
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(false)
                .withIdleMode(MotorMode.COAST)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH);

        abEncoder = new AnalogInput(8); // TODO: Set Absolute Encoder ID
        smctl = new TalonFXWrapper(new TalonFX(IntakeConstants.INTAKE_PIVOT_CAN_ID), DCMotor.getKrakenX60(1), config);
        intakepivot = new Pivot(
                new PivotConfig(smctl)
                        .withStartingPosition(Degrees.of(abEncoder.getValue()))
                        .withWrapping(Degrees.of(0), Degrees.of(360))
                        .withSoftLimits(Degrees.of(-45), Degrees.of(45))
                        .withMOI(Meters.of(0.25), Pounds.of(4))
                        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH));
    }

    public Command setAngle(Angle target) {
        return intakepivot.setAngle(target); // TODO: TEST setMechanismPositionSetpoint
    }

    public Command stop() {
        return intakepivot.set(0);
    }
}
