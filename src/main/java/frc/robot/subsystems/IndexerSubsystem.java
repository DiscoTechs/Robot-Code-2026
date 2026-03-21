package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import frc.robot.Constants;

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
    private SmartMotorController smctl;
    private FlyWheel indexer;

    public IndexerSubsystem() {
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(false)
                .withControlMode(ControlMode.OPEN_LOOP)
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH);

        this.smctl = new TalonFXWrapper(
                new TalonFX(IndexerConstants.MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME)),
                DCMotor.getKrakenX60(1), config);
        this.indexer = new FlyWheel(new FlyWheelConfig(smctl)
                .withDiameter(Inches.of(1.5))
                .withMass(Pounds.of(0.2))
                .withSoftLimit(RPM.of(-6000), RPM.of(6000))
                .withTelemetry("IndexerWheel", TelemetryVerbosity.HIGH));
    }

    public Command index() {
        return indexer.set(-Constants.IndexerConstants.INDEXER_SPEED);
    }

    public Command outtake() {
        return indexer.set(Constants.IndexerConstants.INDEXER_SPEED);
    }

    public Command stop() {
        return indexer.set(0);
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
