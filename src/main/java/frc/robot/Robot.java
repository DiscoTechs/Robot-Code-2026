package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CommandsLogging;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private static Robot instance;
  private SimulatedArena arena;
  private Timer disabledTimer;

  // private CommandXboxController controller = new CommandXboxController(0);
  // private ShooterSubsystem shooter = new ShooterSubsystem();
  // private ClimberSubsystem climber = new ClimberSubsystem();

  public Robot() {
    // Setup Logging
    Logger.recordMetadata("ProjectName", "SwerveCode");
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    CommandScheduler.getInstance().onCommandInitialize(CommandsLogging::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(CommandsLogging::commandEnded);
    CommandScheduler.getInstance().onCommandInterrupt((inted, inting) -> {
      inting.ifPresent(
          (intr) -> CommandsLogging.runningInterrupters.put(intr, inted));
      CommandsLogging.commandEnded(inted);
    });

    // controller.y().whileTrue(climber.climbUp());
    // controller.a().whileTrue(climber.climbDown());
    // controller.x().whileTrue(climber.climbStop());

    // controller.rightTrigger().onTrue(shooter.forward());
    // controller.rightTrigger().onFalse(shooter.stop());

    // controller.leftTrigger().onTrue(Commands.runOnce(() -> shooter.reverse()));
    // controller.leftTrigger().onFalse(Commands.runOnce(() -> shooter.stop()));

    // Init Robot
    disabledTimer = new Timer(); // Create a timer to disable motor brake a few seconds after disable
    m_robotContainer = new RobotContainer();
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    CommandsLogging.logRunningCommands();
    CommandsLogging.logRequiredSubsystems();

    if (Robot.isSimulation()) {
      Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
      Logger.recordOutput("FieldSimulation/FuelPoses", fuelPoses);
    }

    Logger.recordOutput("FieldSimulation/RobotPose", m_robotContainer.getRobotPose());
    Logger.recordOutput("FieldSimulation/TargetPose", m_robotContainer.getSwerveDrive().field.getObject("targetPose").getPose());
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.WHEEL_LOCK_TIME_SEC)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().shutDown();
    // SimulatedArena.overrideInstance(new Arena2026Rebuilt());

    arena = SimulatedArena.getInstance();
    arena.addDriveTrainSimulation(m_robotContainer.getSwerveDrive().getMapleSimDrive().get());
  }

  @Override
  public void simulationPeriodic() {
    arena.simulationPeriodic();
  }
}