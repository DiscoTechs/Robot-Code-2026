package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSlideSubsystem extends SubsystemBase {
    // private SmartMotorController smctl;
    // private Pivot intakeSlide;
    // private AnalogInput abEncoder;
    private TalonFX motor;

    private final DigitalInput inside = new DigitalInput(0);
    private final DigitalInput outside = new DigitalInput(1);

    public IntakeSlideSubsystem() {
        // abEncoder = new AnalogInput(3); // TODO: Set Absolute Encoder ID
        motor = new TalonFX(IntakeConstants.INTAKE_SLIDE_CAN_ID, new CANBus("CANivore 1"));

        // motor.configAllSettings(new TalonSRXConfiguration().)

        // SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
        // .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
        // .withStatorCurrentLimit(Amps.of(40))
        // .withMotorInverted(false)
        // .withIdleMode(MotorMode.COAST)
        // .withControlMode(ControlMode.CLOSED_LOOP)
        // // .withExternalEncoder(abEncoder)
        // .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH);

        // smctl = new TalonWrapper(), DCMotor.getAndymarkRs775_125(1), config);
        // intakepivot = new Pivot(
        // new PivotConfig(smctl)
        // // .withStartingPosition(Degrees.of(abEncoder.getValue() * 360))
        // .withWrapping(Degrees.of(0), Degrees.of(360))
        // // .withSoftLimits(Degrees.of(-45), Degrees.of(45))
        // .withMOI(Meters.of(0.25), Pounds.of(4))
        // .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH));
    }

    public Command forward() {
        return Commands.run(
            () -> motor.set(0.25), this)
                .until(() -> !outside.get())
                .andThen(() -> motor.set(0));
    }

    public Command back() {
        return Commands.run(() -> motor.set(-0.25), this)
                .until(() -> !inside.get())
                .andThen(() -> motor.set(0));
    }

    public Command stop() {
        return Commands.runOnce(() -> motor.set( 0), this);
    }

//     public void periodic(){
//     if (intakeSlide != null) {
// System.out.println("Outside switch = " + outside.get());    intakeSlide.getAngle().in(Degrees);
//     intakeSlide.updateTelemetry();
//     }
//     }
}
