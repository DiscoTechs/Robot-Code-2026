package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import frc.robot.Constants.KickerConstants;
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


public class KickerSubsystem extends SubsystemBase {
    private final SmartMotorController smctl;
    private final FlyWheel kicker;

    private final double DEFAULT_SPEED = 0.85;

    public KickerSubsystem() {
        SmartMotorControllerConfig config = new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
            .withStatorCurrentLimit(Amps.of(20))
            .withMotorInverted(false)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH);

        this.smctl = new TalonFXWrapper(new TalonFX(KickerConstants.KICKER_CAN_ID), DCMotor.getKrakenX60(1), config);
        this.kicker = new FlyWheel(new FlyWheelConfig(smctl)
            .withDiameter(Inches.of(1.5))
            .withMass(Pounds.of(0.2))
            .withSoftLimit(RPM.of(-6000), RPM.of(6000))
            .withTelemetry("KickerWheel", TelemetryVerbosity.HIGH));
    }

    public Command forward() {
        return kicker.set(DEFAULT_SPEED);
    }

    public Command reverse() {
        return kicker.set(-DEFAULT_SPEED);
    }

    public Command setSpeed(double speed) {
        return kicker.set(speed);
    }

    public Command stop() {
        return kicker.set(0.0);
    }

    @Override
    public void periodic() {
        if (kicker != null) {
            kicker.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (kicker != null) {
            kicker.simIterate();
        }
    }
}
