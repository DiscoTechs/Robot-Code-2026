package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoCenter;
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

    public void recordTelemetry() {
        Logger.recordOutput("RobotPOV", robotRelative ? "Robot" : "Field");
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
    }

    public void init() {
        SwerveInputStream driveAngularVelocity = SwerveInputStream
                .of(drivebase.getSwerveDrive(), () -> controller.getLeftY() * (isRedAlliance() ? -1 : 1),
                        () -> controller.getLeftX() * (isRedAlliance() ? -1 : 1))
                .withControllerRotationAxis(() -> controller.getRightX() * (isRedAlliance() ? 1 : -1))
                .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
                .deadband(OperatorConstants.JOYSTICK_DEADBAND)
                .robotRelative(() -> robotRelative)
                .allianceRelativeControl(() -> !robotRelative);

        recordTelemetry();
        drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

        // if (RobotBase.isSimulation()) {
        // drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveDirectAngleKeyboard));

        // Pose2d target = new Pose2d(new Translation2d(1, 4),
        // Rotation2d.fromDegrees(90));
        // // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
        // driveDirectAngleKeyboard.driveToPose(() -> target,
        // new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
        // new ProfiledPIDController(5, 0, 0,
        // new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));

        // controller.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        // controller.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
        // controller.button(2).whileTrue(Commands.runEnd(() ->
        // driveDirectAngleKeyboard.driveToPoseEnabled(true),
        // () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
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

            // controller.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
            // drivebase).repeatedly())
            controller.leftBumper().whileTrue(new AutoCenter(drivebase));
            controller.rightBumper().onTrue(Commands.runOnce(() -> {
                recordTelemetry();
                robotRelative = !robotRelative;
            }));

            controller.leftTrigger()
                    .onTrue(Commands.runOnce(() -> drivebase.setHalfSpeed(true)))
                    .onFalse(Commands.runOnce(() -> drivebase.setHalfSpeed(false)));
        }
    }
}