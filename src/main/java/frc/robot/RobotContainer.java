// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.RotateCommand;
import frc.robot.inputs.DriverInput;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

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
  public final DriverInput driverInput = new DriverInput(Constants.OperatorConstants.PORT, drivebase);

  private final SendableChooser<Command> autoChooser;
//   private Alliance currentAlliance = Alliance.Red;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (!Robot.isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Setup Inputs
    driverInput.init(); // Configure our controller + send input to swervedrive

    // Register Commands
    NamedCommands.registerCommand("driveBackwards", drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
    NamedCommands.registerCommand("driveForwards", drivebase.driveForward().withTimeout(2).withName("Auto.driveForwards"));

    // Setup Auto
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(10)); // Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2).andThen()
    autoChooser.addOption("Rotate 45", new RotateCommand(drivebase));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (autoChooser.getSelected() == null) {
      RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }
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