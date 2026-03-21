package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.results.RawFiducial;
import limelight.networktables.PoseEstimate;

public class AutoCenter extends Command {
    private final PIDController turnPID = new PIDController(0.03, 0, 0.001);
    private final SwerveSubsystem drivetrain;
    private boolean hasTarget = false;

    public AutoCenter(SwerveSubsystem dt) {
        this.drivetrain = dt;
        this.turnPID.setTolerance(1.0);

        addRequirements(dt);
    }

    @Override
    public void initialize() {
        this.turnPID.reset();
    }

    @Override
    public void execute() {
        Optional<PoseEstimate> visionEstimate = drivetrain
            .getLimelight()
            .createPoseEstimator(EstimationMode.MEGATAG2)
            .getPoseEstimate();

        hasTarget = false;
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            if (poseEstimate.tagCount > 0) {
                RawFiducial tag = poseEstimate.rawFiducials[0];
                if (tag == null) { return; }
                hasTarget = true;

                double rotation = turnPID.calculate(tag.txnc, 0);
                System.out.println("TagX:" + tag.txnc);
                System.out.println("R: " + rotation);

                drivetrain.getSwerveDrive().drive(new Translation2d(0, 0), rotation, false, false);
            }
        });
    }

    @Override
    public boolean isFinished() {
        System.out.println(turnPID.atSetpoint());
        return hasTarget && turnPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.getSwerveDrive()
            .drive(new Translation2d(0, 0), 0, false, false);
    }
}
