// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RotateCommand;
import frc.robot.inputs.DriverInput;
import frc.robot.inputs.OperatorInput;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OperatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();

  private final OperatorSubsystem operatorSubsystem = new OperatorSubsystem(climber, indexer, shooter);
  private final OperatorInput operatorInput = new OperatorInput(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT, drivebase, operatorSubsystem);
  private final DriverInput driverInput = new DriverInput(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT, drivebase);

  private final SendableChooser<Command> autoChooser;
  private Alliance currentAlliance = Alliance.Red;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (!Robot.isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Setup Inputs
    driverInput.init(); // Configure our controller to send input to swervedrive
    operatorInput.init(); // Configure our controller to send input to operator subsystems

    // Register Commands
    NamedCommands.registerCommand("driveBackwards", drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
    NamedCommands.registerCommand("driveForwards", drivebase.driveForward().withTimeout(2).withName("Auto.driveForwards"));

    climber.setDefaultCommand(climber.moveTo(Meters.of(0)));

    // Aliance
    onAllianceChanged(getAlliance());
    new Trigger(() -> getAlliance() != currentAlliance)
        .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoringDisable(true));

    // Setup Auto
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(10));
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

  // Alliance
  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private boolean isInAllianceZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  private boolean isOnAllianceOutpostSide() {
    Alliance alliance = getAlliance();
    Distance midLine = Inches.of(158.84375);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midLine)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midLine)) {
      return true;
    }

    return false;
  }

  private void onAllianceChanged(Alliance alliance) {
    currentAlliance = alliance;
    System.out.println("Alliance changed to: " + alliance);
  }
}