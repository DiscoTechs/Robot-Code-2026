package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimberSubsystem extends SubsystemBase {
    private SmartMotorController smctl;
    private Elevator elevator;
    private TalonFX motor;

    private final DigitalInput topLimit = new DigitalInput(0);
    private final DigitalInput bottomLimit = new DigitalInput(1);

    public ClimberSubsystem() {
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withMechanismCircumference(Inches.of(1.5).times(Math.PI))
                .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
                .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
                .withFeedforward(new ElevatorFeedforward(0, 0, 0))
                .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
                .withMotorInverted(true)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(40))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withTelemetry("ClimberMotor", TelemetryVerbosity.HIGH);

        this.motor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);
        this.smctl = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), config);
        this.elevator = new Elevator(
                new ElevatorConfig(smctl)
                        .withMass(ClimberConstants.MASS)
                        .withStartingHeight(Meters.of(0))
                        .withSoftLimits(Meters.of(0), Meters.of(0.75))
                        .withTelemetry("Climber", TelemetryVerbosity.HIGH));
    }

    public Command climbUp() {
        if (topLimit.get()) {
            return elevator.set(0);
        }

        return elevator.set(0.5)
            .until(() -> topLimit.get())
            .andThen(stop());
    }

    public Command climbDown() {
        if (bottomLimit.get()) {
            return elevator.set(0);
        }

        return elevator.set(-0.5)
            .until(() -> bottomLimit.get())
            .andThen(stop());
    }

    public Command stop() {
        return elevator.set(0);
    }

    @Override
    public void periodic() {
        if (elevator != null) {
            elevator.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (elevator != null) {
            elevator.simIterate();
        }
    }
}
