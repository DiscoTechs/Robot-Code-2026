package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
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
    private final SmartMotorController smctl;
    private final Pivot turret;

    private final DutyCycleEncoder abEncoder;
    private Limelight limelight;

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

        // limelight = new Limelight("limelight2");
        abEncoder = new DutyCycleEncoder(0); // TODO: Set Absolute Encoder ID
        smctl = new TalonFXWrapper(new TalonFX(TurretConstants.TURRET_MOTOR_CAN_ID), DCMotor.getKrakenX60(1), config);
        turret = new Pivot(
            new PivotConfig(smctl)
                .withStartingPosition(Degrees.of(abEncoder.get() * 360))
                .withSoftLimits(Degrees.of(-80), Degrees.of(80))
                .withMOI(Meters.of(0.25), Pounds.of(4))
                .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH));

    //   limelight.getSettings()
    //       .withLimelightLEDMode(LEDMode.PipelineControl)
    //       .withCameraOffset(new Pose3d(
    //             Inches.of(0).in(Meters),
    //             Inches.of(0).in(Meters),
    //             Inches.of(0).in(Meters),
    //             new Rotation3d(0, Degrees.of(0).in(Radians), Degrees.of(0).in(Radians))))
    //       .save();
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
