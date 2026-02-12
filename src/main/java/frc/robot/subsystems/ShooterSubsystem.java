package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.mechanisms.velocity.FlyWheel;
import yams.mechanisms.config.FlyWheelConfig;
import yams.gearing.MechanismGearing;
import yams.gearing.GearBox;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.system.plant.DCMotor;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX motor = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_CAN_ID);
    private SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(0.00015, 0.0, 0.0, RPM.of(6000), RotationsPerSecondPerSecond.of(2500))
            .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH);
    
    private SmartMotorController smctl = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), config);
    private FlyWheel shooter = new FlyWheel(
        new FlyWheelConfig(smctl)
                    .withDiameter(Inches.of(1.5))
                    .withMass(Pounds.of(0.5))
                    .withTelemetry("Indexer", TelemetryVerbosity.HIGH));

    public ShooterSubsystem() {}

    public Command forward() {
        return shooter.set(0.5);
    }

    public Command reverse() {
        return shooter.set(-0.5);
    }

    public Command stop() {
        return shooter.set(0);
    }

    @Override
    public void periodic() { shooter.updateTelemetry(); }

    @Override
    public void simulationPeriodic() { shooter.simIterate(); }
}
