package frc.robot.commands;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.results.RawFiducial;
import limelight.networktables.PoseEstimate;

public class AutoCenter extends Command {
    // private final PIDController turnPID = new PIDController(0.03, 0, 0.001);
    private final SwerveSubsystem drivetrain;

    public AutoCenter(SwerveSubsystem dt) {
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
                if (tag == null) { return; }

                System.out.println(tag.txnc);
                drivetrain.getSwerveDrive().drive(new Translation2d(0, 0), tag.txnc, false, false);
            }
        });
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
