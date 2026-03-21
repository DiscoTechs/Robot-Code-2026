// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.RotateCommand;
import frc.robot.inputs.DriverInput;
import frc.robot.inputs.OperatorInput;
// import frc.robot.inputs.OperatorInput;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OperatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
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

    private boolean SHOOTER_ENABLED = true;
    private boolean INDEXER_ENABLED = true;
    private boolean INTAKE_SLIDE_ENABLED = true;
    private boolean INTAKE_ENABLED = true;
    private boolean CONVEYOR_ENABLED = true;

    private IntakeSlideSubsystem intakeSlide;
    private ConveyorSubsystem conveyor;
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;

    private final OperatorSubsystem operatorSubsystem;
    private final OperatorInput operatorInput;
    private final DriverInput driverInput;

    private final LoggedDashboardChooser<Command> autoChooser;
    // private Alliance currentAlliance = Alliance.Red;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (!Robot.isReal()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        try {
            this.shooter = SHOOTER_ENABLED ? new ShooterSubsystem() : null;
        } catch (Error err) {
            this.shooter = null;

            System.out.println("WARNING: Shooter failed to init.");
            System.out.println(err);
        }

        try {
            this.indexer = INDEXER_ENABLED ? new IndexerSubsystem() : null;
        } catch (Error err) {
            this.indexer = null;

            System.out.println("WARNING: Indexer failed to init.");
            System.out.println(err);
        }

        try {
            this.intakeSlide = INTAKE_SLIDE_ENABLED ? new IntakeSlideSubsystem() : null;
        } catch (Error err) {
            this.intakeSlide = null;

            System.out.println("WARNING: IntakeSlide failed to init.");
            System.out.println(err);
        }

        try {
            this.intake = INTAKE_ENABLED ? new IntakeSubsystem() : null;
        } catch (Error err) {
            this.intake = null;

            System.out.println("WARNING: Intake failed to init.");
            System.out.println(err);
        }

        try {
            this.conveyor = CONVEYOR_ENABLED ? new ConveyorSubsystem() : null;
        } catch (Error err) {
            this.conveyor = null;

            System.out.println("WARNING: Conveyor failed to init.");
            System.out.println(err);
        }

        this.operatorSubsystem = new OperatorSubsystem(indexer, shooter, intakeSlide, intake, conveyor);

        // Setup Inputs
        driverInput = new DriverInput(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT, drivebase);
        driverInput.init(); // Configure our controller to send input to swervedrive

        operatorInput = new OperatorInput(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT, operatorSubsystem);
        operatorInput.init(); // Configure our controller to send input to operator subsystems

        // Register Commands
        NamedCommands.registerCommand("driveBackwards",
                drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
        NamedCommands.registerCommand("driveForwards",
                drivebase.driveForward().withTimeout(2).withName("Auto.driveForwards"));

        // Setup Auto
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
        autoChooser.addDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(3));
        autoChooser.addOption("Drive Backward", drivebase.driveBackwards().withTimeout(3));
        autoChooser.addOption("Rotate 90", new RotateCommand(drivebase));

        RobotModeTriggers.disabled().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        if (autoChooser.get() != null) {
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        }

        // NamedCommands.registerCommand("shoot", Commands.parallel(
        // shooter.forward().asProxy(),
        // (new WaitCommand(0.5).andThen(kicker::forward)).asProxy()
        // // (new WaitCommand(0.5).andThen(indexer::forward)).asProxy(),
        // // new
        // WaitCommand(3).andThen(indexer::stop).andThen(kicker::stop).andThen(shooter::stop))
        // ));
        NamedCommands.registerCommand("intake", Commands.parallel(
            intake.forward().asProxy(),
            conveyor.forward().asProxy()
        ));

        NamedCommands.registerCommand("outake", Commands.parallel(
            intake.reverse().asProxy(),
            conveyor.reverse().asProxy()
        ));

        NamedCommands.registerCommand("intakeSlideExtend", intakeSlide.extend());
        NamedCommands.registerCommand("intakeSlideRetract", intakeSlide.retract());
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
}