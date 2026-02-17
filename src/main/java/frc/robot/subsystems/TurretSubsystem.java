package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
    private SmartMotorController smctl;
    private TalonFX motor;
    private Pivot turret;

    public TurretSubsystem() {
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(12)))
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(false)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH);

        this.motor = new TalonFX(TurretConstants.TURRET_MOTOR_CAN_ID);
        this.smctl = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), config);
        this.turret = new Pivot(
            new PivotConfig(smctl)
                .withStartingPosition(Degrees.of(0)) // TODO: Use absolute encoder get degrees
                .withWrapping(Degrees.of(0), Degrees.of(360))
                .withSoftLimits(Degrees.of(-135), Degrees.of(135))
                .withMOI(Meters.of(0.25), Pounds.of(4))
                .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH));
    }

    public Command setAngle(Angle angle) {
        return turret.setAngle(angle);
    }

    public Angle getAngle() {
        return turret.getAngle();
    }

    public Command set(double speed) {
        return turret.set(speed);
    }

    public Command stop() {
        return turret.set(0);
    }

    @Override
    public void periodic() {
        if (turret != null) {
            turret.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (turret != null) {
            turret.simIterate();
        }
    }
}
