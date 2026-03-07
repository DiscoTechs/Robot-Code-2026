// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.io.File;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.RotateCommand;
import frc.robot.inputs.DriverInput;
import frc.robot.inputs.OperatorInput;
// import frc.robot.inputs.OperatorInput;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.OperatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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

    private boolean CLIMBER_ENABLED = true;
    private boolean SHOOTER_ENABLED = true;
    private boolean INDEXER_ENABLED = false;
    private boolean TURRET_ENABLED = false;
    private boolean KICKER_ENABLED = true;
    private boolean INTAKE_PIVOT_ENABLED = false;
    private boolean INTAKE_ENABLED = false;

    private final ClimberSubsystem climber = CLIMBER_ENABLED ? new ClimberSubsystem() : null;
    private final ShooterSubsystem shooter = SHOOTER_ENABLED ? new ShooterSubsystem() : null;
    private final IndexerSubsystem indexer = INDEXER_ENABLED ? new IndexerSubsystem() : null;
    private final TurretSubsystem turret = TURRET_ENABLED ? new TurretSubsystem() : null;
    private final KickerSubsystem kicker = KICKER_ENABLED ? new KickerSubsystem() : null;
    private final IntakePivotSubsystem intakePivot = INTAKE_PIVOT_ENABLED ? new IntakePivotSubsystem () : null;
    private final IntakeSubsystem intake = INTAKE_ENABLED ? new IntakeSubsystem() : null;
    
    private final OperatorSubsystem operatorSubsystem = new OperatorSubsystem(climber, indexer, shooter, turret, intakePivot, intake, kicker);
    private final OperatorInput operatorInput = new OperatorInput(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT, operatorSubsystem);
    private final DriverInput driverInput = new DriverInput(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT, drivebase);

    private final LoggedDashboardChooser<Command> autoChooser;
    // private Alliance currentAlliance = Alliance.Red;

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
        NamedCommands.registerCommand("driveBackwards",
                drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
        NamedCommands.registerCommand("driveForwards",
                drivebase.driveForward().withTimeout(2).withName("Auto.driveForwards"));
        // climber.setDefaultCommand(climber.climbUp());

        // Aliance
        // onAllianceChanged(getAlliance());
        // new Trigger(() -> getAlliance() != currentAlliance)
        //         .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoringDisable(true));

        // Setup Auto
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
        autoChooser.addDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(3));
        autoChooser.addOption("Drive Backward", drivebase.driveBackwards().withTimeout(3));
        autoChooser.addOption("Rotate 45", new RotateCommand(drivebase));

        if (autoChooser.get() != null) {
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        }
   
        int[] tags = { 16, 32 };
        NamedCommands.registerCommand("climbDriveToTarget", new DriveToTarget(drivebase, tags));
        NamedCommands.registerCommand("shoot", Commands.parallel(
            shooter.forward().asProxy(),
            (new WaitCommand(0.5).andThen(kicker::forward)).asProxy()
            // (new WaitCommand(0.5).andThen(indexer::forward)).asProxy(),
            // new WaitCommand(3).andThen(indexer::stop).andThen(kicker::stop).andThen(shooter::stop))
        ));   

        NamedCommands.registerCommand("climb", climber.climbDown()
            .andThen(climber.climbUp().withTimeout(2)
            .andThen(climber.climbDown())));
        NamedCommands.registerCommand("climbUp", climber.climbUp());
        NamedCommands.registerCommand("climbDown", climber.climbDown());

        // NamedCommands.registerCommand("turretAngle0", turret.setAngle(Degrees.of(0)));
        // NamedCommands.registerCommand("turretAngle45", turret.setAngle(Degrees.of(45)));
        // NamedCommands.registerCommand("turretAngle90", turret.setAngle(Degrees.of(90)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
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
    // private Alliance getAlliance() {
    //     return DriverStation.getAlliance().orElse(Alliance.Red);
    // }

    // private boolean isInAllianceZone() {
    //     Alliance alliance = getAlliance();
    //     Distance blueZone = Inches.of(182);
    //     Distance redZone = Inches.of(469);

    //     if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) {
    //         return true;
    //     } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) {
    //         return true;
    //     }

    //     return false;
    // }

    // private boolean isOnAllianceOutpostSide() {
    //     Alliance alliance = getAlliance();
    //     Distance midLine = Inches.of(158.84375);

    //     if (alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midLine)) {
    //         return true;
    //     } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midLine)) {
    //         return true;
    //     }

    //     return false;
    // }

    // private void onAllianceChanged(Alliance alliance) {
    //     currentAlliance = alliance;
    //     System.out.println("Alliance changed to: " + alliance);
    // }
}