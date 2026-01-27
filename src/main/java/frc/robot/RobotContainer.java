// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.RotateCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;
  private Alliance currentAlliance = Alliance.Red;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutoCommands();

    // If sim then don't care about joystick
    if (!Robot.isReal() || true) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2).andThen()

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(10));
    autoChooser.addOption("Absolute Drive", new AbsoluteDriveAdv(drivebase, null, null, null, null, null, null, null));
    autoChooser.addOption("Rotate 45", new RotateCommand(drivebase));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (autoChooser.getSelected() == null) {
      RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation()) {
      System.out.println("Using keyboard drive control");
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(5, 0, 0,
              new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));

      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  private void configureAutoCommands() {
    NamedCommands.registerCommand("driveBackwards", drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
    NamedCommands.registerCommand("driveForwards", drivebase.driveForward().withTimeout(2).withName("Auto.driveForwards"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public SwerveDrive getSwerveDrive() {
    return drivebase.getSwerveDrive();
  }

  public Pose2d getRobotPose() {
    return drivebase.getPose();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}