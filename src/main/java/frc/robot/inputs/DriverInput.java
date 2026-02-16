package frc.robot.inputs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToTarget;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriverInput {
    public static CommandXboxController controller;
    private SwerveSubsystem drivebase;

    SwerveInputStream driveAngularVelocity = SwerveInputStream
        .of(drivebase.getSwerveDrive(), () -> controller.getLeftY() * -1, () -> controller.getLeftX() * -1)
        .withControllerRotationAxis(() -> -controller.getRightX())
        .deadband(OperatorConstants.JOYSTICK_DEADBAND)
        .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
        .robotRelative(false)
        .allianceRelativeControl(true);
    
    SwerveInputStream driveAngularVelocityRobotRelative = driveAngularVelocity
        .robotRelative(true)
        .allianceRelativeControl(false);

    public DriverInput(int ctlrPort, SwerveSubsystem ss) {
        controller = new CommandXboxController(ctlrPort);
        drivebase = ss;
    }

    public void init() {
        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

        if (DriverStation.isTest()) {
            controller.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            controller.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            controller.back().whileTrue(drivebase.centerModulesCommand());
            controller.leftBumper().onTrue(Commands.none());
            controller.rightBumper().onTrue(Commands.none());
        } else if (RobotBase.isSimulation()) {
            // drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngleKeyboard));

            // Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
            // // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
            // driveDirectAngleKeyboard.driveToPose(() -> target,
            //         new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
            //         new ProfiledPIDController(5, 0, 0,
            //                 new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));

            // controller.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
            // controller.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            // controller.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
            //         () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
        } else {
            controller.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
            controller.back().whileTrue(Commands.none());

            controller.leftBumper().toggleOnTrue(Commands.runOnce(() -> drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocityRobotRelative))));
            controller.leftBumper().toggleOnFalse(Commands.runOnce(() -> drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity))));

            // controller.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            controller.rightBumper().whileTrue(new DriveToTarget(drivebase));
        }
    }
}