package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.results.RawFiducial;
import limelight.networktables.PoseEstimate;

// TODO: Verify that this code works
public class DriveToTarget extends Command {
    private final PIDController turnPID = new PIDController(0.03, 0, 0.001);
    private final SwerveSubsystem drivetrain;
    private double lastDistance = 10;

    public DriveToTarget(SwerveSubsystem dt) {
        this.drivetrain = dt;

        addRequirements(dt);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Optional<PoseEstimate> visionEstimate = drivetrain.getLimelight().createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            if (poseEstimate.tagCount > 0) {
                RawFiducial tag = poseEstimate.rawFiducials[0];
                lastDistance = tag.distToCamera;

                double rotation = turnPID.calculate(tag.txnc, 0);
                if (Math.abs(tag.txnc) < 1.0) {
                    rotation = 0;
                }

                drivetrain.getSwerveDrive().drive(new Translation2d(0.75, 0), rotation, false, false);
                Logger.recordOutput("Limelight/targetDistance", lastDistance);
                Logger.recordOutput("Limelight/targetX", tag.txnc);
            } else {
                Logger.recordOutput("Limelight/targetDistance", 0);
            }
        });
    }

    @Override
    public boolean isFinished() {
        return lastDistance < 1.15;
    }
}
