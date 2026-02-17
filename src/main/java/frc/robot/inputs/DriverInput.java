package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.DriveToTarget;
import swervelib.SwerveInputStream;

public class DriverInput {
    public static CommandXboxController controller;
    private SwerveSubsystem drivebase;
    private boolean robotRelative = false;

    public DriverInput(int ctlrPort, SwerveSubsystem ss) {
        controller = new CommandXboxController(ctlrPort);
        drivebase = ss;
    }

    public void init() {
        SwerveInputStream driveAngularVelocity = SwerveInputStream
                .of(drivebase.getSwerveDrive(), () -> controller.getLeftY() * -1, () -> controller.getLeftX() * -1)
                .withControllerRotationAxis(() -> controller.getRightX() * -1)
                .deadband(OperatorConstants.JOYSTICK_DEADBAND)
                .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
                .robotRelative(() -> robotRelative)
                .allianceRelativeControl(() -> !robotRelative);

        Logger.recordOutput("RobotPOV", robotRelative ? "Robot" : "Field");
        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

        // if (RobotBase.isSimulation()) {
        //     drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngleKeyboard));

        //     Pose2d target = new Pose2d(new Translation2d(1, 4),
        //     Rotation2d.fromDegrees(90));
        //     // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
        //     driveDirectAngleKeyboard.driveToPose(() -> target,
        //     new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
        //     new ProfiledPIDController(5, 0, 0,
        //     new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));

        //     controller.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        //     controller.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
        //     controller.button(2).whileTrue(Commands.runEnd(() ->
        //     driveDirectAngleKeyboard.driveToPoseEnabled(true),
        //     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
        // }

        if (DriverStation.isTest()) {
            controller.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            controller.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            controller.back().whileTrue(drivebase.centerModulesCommand());
            controller.leftBumper().onTrue(Commands.none());
            controller.rightBumper().onTrue(Commands.none());
        } else {
            controller.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
            controller.back().whileTrue(Commands.none());

            controller.rightBumper().onTrue(Commands.runOnce(() -> {
                Logger.recordOutput("RobotPOV", robotRelative ? "Robot" : "Field");
                robotRelative = !robotRelative;
            }));
            controller.leftBumper().whileTrue(new DriveToTarget(drivebase));
            // controller.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        }
    }
}