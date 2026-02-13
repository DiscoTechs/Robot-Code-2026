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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
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

  private SmartMotorController smctl;
  private Elevator elevator;

  public ClimberSubsystem() {
    this.smctl = new TalonFXWrapper(new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID), DCMotor.getKrakenX60(1), smcConfig);
    this.elevator = new Elevator(
        new ElevatorConfig(smctl)
            .withMass(ClimberConstants.MASS)
            .withStartingHeight(Meters.of(0))
            .withSoftLimits(Meters.of(0), Meters.of(0.75))
            .withTelemetry("Climber", TelemetryVerbosity.HIGH));
  }

  public Command moveTo(Distance height) {
    return elevator.setHeight(height);
  }

  // public Command sysId() {
  // return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  // }

  public Command climbUp() {
    return elevator.set(0.5);
  }

  public Command climbDown() {
    return elevator.set(-0.5);
  }

  public Command climbStop() {
    System.out.println("STOP: " + elevator.getHeight());
    return elevator.set(0);
  }

  @Override
  public void periodic() {
    elevator.updateTelemetry();

    System.out.println("Height:" + elevator.getHeight());
  }

  @Override
  public void simulationPeriodic() {
    elevator.simIterate();
  }
}
