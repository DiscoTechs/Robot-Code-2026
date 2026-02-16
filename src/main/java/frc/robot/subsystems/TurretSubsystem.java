package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.gearing.GearBox;
import yams.gearing.Sprocket;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.LEDMode;

import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.pipeline.NeuralClassifier;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private Pivot turretPivot;
    
    // Turret camera using YALL.
    private Limelight turretCam;
    private LimelightPoseEstimator turretPoseEstimator;
    private Pose3d turretCamOffset = new Pose3d(
        Inches.of(5).in(Meters),
        Inches.of(5).in(Meters),
        Inches.of(5).in(Meters),
        Rotation3d.kZero
    );
    
    private double angleToHub;

    // Constructor.
    public TurretSubsystem() {

        // Init turret motor controllor.
        SmartMotorControllerConfig turretMotorCfg = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            //.withMechanismCircumference(Inches.of(1.5).times(Math.PI))
            .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withSimClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.0), new Sprocket(22, 200)))
            .withMotorInverted(true)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH);
    
        SmartMotorController turretMotor = new TalonFXWrapper(
            new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_CAN_ID),
            DCMotor.getKrakenX60(1), 
            turretMotorCfg
        );

        turretPivot = new Pivot(
            new PivotConfig(turretMotor)
            .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
            //.withWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled bc the pivot can spin infinitely
            .withHardLimit(Degrees.of(0), Degrees.of(270)) // Hard limit bc wiring prevents infinite spinning
            .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH) // Telemetry
            .withMOI(Meters.of(0.25), Pounds.of(4)) // MOI Calculation
        );
        
         // Zero the turret.
        setAngle(0);

        // Limelight on the turret.
        turretCam = new Limelight("turretcam");
        turretCam.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(turretCamOffset)
            .save();

        turretPoseEstimator = turretCam.createPoseEstimator(EstimationMode.MEGATAG2);

    } //constructor

    public Command turnToMax() {
        return runOnce(
            () -> setAngle(270)
        );
    }

    public Command turnToMin() {
        return runOnce(
            () -> setAngle(0)
        );
    }
    public void setAngle(double angle) {
        turretPivot.setAngle(Angle.ofBaseUnits(angle, Degrees));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(0);
    }

    public void setZero() {}

    public void stop() {}

    
    @Override
    public void periodic() {
//         if (Constants.Limelight.ENABLED) {
//             turretPoseEstimator.getPoseEstimate().ifPresent(
//                 (PoseEstimate poseEstimate) -> {
//                     if (poseEstimate.tagCount > 0) {
//                         angleToHub = poseEstimate.pose.toPose2d().minus(redHub.toPose2d()).getTranslation().getNorm();
//                     }
//                 }
//             )

//             turretCam.getLatestResults().ifPresent((LimelightResults result) -> {
//     for (NeuralClassifier object : result.targets_Fiducials)
//     {
//         // Classifier says its a coral.
//         if (object.className.equals("coral"))
//         {
//             // Check pixel location of coral.
//             if (object.ty > 2 && object.ty < 1)
//             {
//             // Coral is valid! do stuff!
//             }
//         }
//     }
// });
//         }   
    }

}

