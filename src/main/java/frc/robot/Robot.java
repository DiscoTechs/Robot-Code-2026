package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import frc.robot.util.CommandsLogging;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private static Robot instance;
  private SimulatedArena arena;
  private Timer disabledTimer;

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

    // Init Robot
    disabledTimer = new Timer(); // Create a timer to disable motor brake a few seconds after disable
    m_robotContainer = new RobotContainer();
    instance = this;

    // absoluteEncoder = new AnalogInput(absoluteEncoderId);
    // SparkMax driveMotor = new SparkMax(1, MotorType.kBrushless);
    // turnMotor = new SparkMax(turningId, MotorType.kBrushless);
    // driveEncoder = driveMotor.getEncoder();

    // SparkMaxConfig driveConfig = new SparkMaxConfig();
    // driveConfig.smartCurrentLimit(40);
    // driveConfig.inverted(false);

    // SparkMaxConfig turnConfig = new SparkMaxConfig();
    // turnConfig.smartCurrentLimit(20);
    // turnConfig.inverted(false);

    // driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    // driveMotor.set(0.1);
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
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
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
      m_autonomousCommand.schedule();
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
    SimulatedArena.overrideInstance(new Arena2026Rebuilt());

    arena = SimulatedArena.getInstance();
    arena.addDriveTrainSimulation(m_robotContainer.getSwerveDrive().getMapleSimDrive().get());
  }

  @Override
  public void simulationPeriodic() {
    arena.simulationPeriodic();
  }
}