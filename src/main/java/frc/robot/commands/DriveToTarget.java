package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.networktables.LimelightResults;
import limelight.networktables.target.AprilTagFiducial;

// TODO: Verify that this code works
public class DriveToTarget extends Command {
    private final SwerveSubsystem drivetrain;

    public DriveToTarget(SwerveSubsystem dt) {
        this.drivetrain = dt;

        addRequirements(dt);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        LimelightResults result = drivetrain.getLimelight().getLatestResults().get();
        AprilTagFiducial tag = result.targets_Fiducials[0];
        Pose2d targetPose = tag.getTargetPose_RobotSpace2D();

        Logger.recordOutput("Limelight/targetPose", targetPose);
        drivetrain.driveToPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
