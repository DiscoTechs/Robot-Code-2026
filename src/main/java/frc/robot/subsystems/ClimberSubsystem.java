package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

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
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
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
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

  private SmartMotorController smctl = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), smcConfig);
  private Elevator elevator = new Elevator(
      new ElevatorConfig(smctl)
          .withMass(ClimberConstants.MASS)
          .withStartingHeight(ClimberConstants.STARTING_HEIGHT)
          .withHardLimits(Meters.of(0), Meters.of(.762))
          .withTelemetry("Elevator", TelemetryVerbosity.HIGH));

  public ClimberSubsystem() {}

  public Command setHeight(Distance height) {
    return elevator.setHeight(height);
  }

  // public Command sysId() {
  //   return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  // }

  public Command climbUp() {
    return elevator.set(1);
  }

  public Command climbDown() {
    return elevator.set(-1);
  }

  public Command climbSTOP() {
    return elevator.set(0);
  }

  @Override
  public void periodic() {
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    elevator.simIterate();
  }
}
