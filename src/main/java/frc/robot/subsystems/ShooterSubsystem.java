package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.SmartMotorController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.gearing.MechanismGearing;
import yams.gearing.GearBox;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private SmartMotorController smctl;
    private FlyWheel shooter;
    private TalonFX motor;

    public ShooterSubsystem() {
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(true)
                .withIdleMode(MotorMode.COAST)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(0.01, 0.0, 0.0, RPM.of(6000), RotationsPerSecondPerSecond.of(4500))
                .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

        motor = new TalonFX(Constants.ShooterConstants.MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
        smctl = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), config);
        shooter = new FlyWheel(
                new FlyWheelConfig(smctl)
                        .withDiameter(Inches.of(1.5))
                        .withMass(Pounds.of(0.5))
                        .withSoftLimit(RPM.of(0), RPM.of(1000))
                        .withTelemetry("Indexer", TelemetryVerbosity.HIGH));
    }

    public AngularVelocity getSpeed() { return shooter.getSpeed(); }
    public Command setShooterSpeed(AngularVelocity speed) { return shooter.run(speed); }
    public Command stop() { return shooter.set(0); }

    public Command set(double speed) { return shooter.set(speed); }

    @Override
    public void periodic() { shooter.updateTelemetry(); }

    @Override
    public void simulationPeriodic() { shooter.simIterate(); }
}