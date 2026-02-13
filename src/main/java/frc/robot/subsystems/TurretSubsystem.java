package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    // Turret motor controller using YAMS. 
    private SmartMotorController turretMotor;

    // Turret camera using YALL.
    private Limelight turretCam;
    private LimelightPoseEstimator turretPoseEstimator;
    private Pose3d turretCamOffset = new Pose3d(
        Inches.of(5).in(Meters),
        Inches.of(5).in(Meters),
        Inches.of(5).in(Meters),
        Rotation3d.kZero
    );
    
    // Constructor.
    public TurretSubsystem() {

        // Init turret motor controllor.
        turretMotor = new TalonFXWrapper(
            new TalonFX(Constants.ShooterConstants.TURRET_MOTOR_CAN_ID),
            DCMotor.getKrakenX60(1), 
            new SmartMotorControllerConfig(this)
                //.withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(true)
                .withIdleMode(MotorMode.BRAKE)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(0.00015, 0.0, 0.0, RPM.of(1000), RotationsPerSecondPerSecond.of(2500))
                .withTelemetry("turretmotor", TelemetryVerbosity.HIGH)
        );

        // Limelight on the turret.
        turretCam = new Limelight("turretcam");
        turretCam.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(turretCamOffset)
            .save();

        turretPoseEstimator = turretCam.createPoseEstimator(EstimationMode.MEGATAG2);
        // Zero the turret.
        turretMotor.setPosition(Angle.ofBaseUnits(0, Degrees));
    }

    public void setAngle(double angle) {}

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(0);
    }

    public void setZero() {}
    public void stop() {}
}
