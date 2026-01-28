// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RotateCommand extends Command {
  private final SwerveSubsystem swerve;
  private Rotation2d targetHeading;

  public RotateCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Rotation2d currentHeading = swerve.getHeading();
    targetHeading = currentHeading.plus(Rotation2d.fromDegrees(45));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing rotate (Current: " + swerve.getHeading().getDegrees() + "*" + " Target: " + targetHeading.getDegrees() + "*)");

    Translation2d translation = new Translation2d(0, 0);
    double headingX = targetHeading.getSin();
    double headingY = targetHeading.getCos();
  
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0.0, 0.0, headingX, headingY);
    // swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -targetHeading.getAsDouble()), true);
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return Math.abs(swerve.getHeading().minus(targetHeading).getDegrees()) < 1.0;
  }
}