package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private SmartMotorController smctl;
    private Elevator elevator;
    private TalonFX motor;

    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private final double DEFAULT_SPEED = 0.5;

    // private Limelight limelight = new Limelight("limelight2");

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

        this.topLimit = new DigitalInput(0);
        this.bottomLimit = new DigitalInput(1);

        this.motor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID, new CANBus("CANivore 1"));
        this.smctl = new TalonFXWrapper(this.motor, DCMotor.getKrakenX60(1), config);

        this.elevator = new Elevator(
                new ElevatorConfig(smctl)
                        .withMass(ClimberConstants.MASS)
                        .withStartingHeight(Meters.of(0))
                        .withSoftLimits(Meters.of(0), Meters.of(0.75))
                        .withTelemetry("Climber", TelemetryVerbosity.HIGH));

        // limelight.getSettings()
        // .withLimelightLEDMode(LEDMode.PipelineControl)
        // .withCameraOffset(new Pose3d(
        // Inches.of(0).in(Meters),
        // Inches.of(0).in(Meters),
        // Inches.of(0).in(Meters),
        // new Rotation3d(0, Degrees.of(0).in(Radians), Degrees.of(0).in(Radians))))
        // .withImuMode(ImuMode.InternalImuMT1Assist)
        // .withImuAssistAlpha(0.01)
        // .withRobotOrientation(new Orientation3d(
        // drivetrain.getSwerveDrive().getGyro().getRotation3d().plus(new Rotation3d(0,
        // 0, 90)),
        // new AngularVelocity3d(
        // DegreesPerSecond.of(0),
        // DegreesPerSecond.of(0),
        // DegreesPerSecond.of(0))))
        // .save();

        // poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    }

    public Command climbUp() {
        return elevator.set(DEFAULT_SPEED)
                .until(() -> !bottomLimit.get())
                .andThen(stop());
    }

    public Command climbDown() {
        return elevator.set(-DEFAULT_SPEED)
                .until(() -> !topLimit.get()) // figure out how to zero
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
