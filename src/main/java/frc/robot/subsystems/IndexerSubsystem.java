package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IndexerSubsystem extends SubsystemBase {
    private final SmartMotorController smctl;
    private final FlyWheel indexer;

    private final double DEFAULT_SPEED = 0.85;

    public IndexerSubsystem() {
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withStatorCurrentLimit(Amps.of(20))
            .withMotorInverted(false)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH);

        this.smctl = new TalonFXWrapper(new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID, new CANBus("CANivore 1")), DCMotor.getKrakenX60(1), config);
        this.indexer = new FlyWheel(new FlyWheelConfig(smctl)
            .withDiameter(Inches.of(1.5))
            .withMass(Pounds.of(0.2))
            .withSoftLimit(RPM.of(-6000), RPM.of(6000))
            .withTelemetry("IndexerWheel", TelemetryVerbosity.HIGH));
    }

    public Command forward() {
        return indexer.set(DEFAULT_SPEED);
    }

    public Command reverse() {
        return indexer.set(-DEFAULT_SPEED);
    }

    public Command setSpeed(double speed) {
        return indexer.set(speed);
    }

    public Command stop() {
        return indexer.set(0.0);
    }

    @Override
    public void periodic() {
        if (indexer != null) {
            indexer.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (indexer != null) {
            indexer.simIterate();
        }
    }
}
