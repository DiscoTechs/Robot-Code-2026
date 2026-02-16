package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.OperatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class OperatorInput {
    public static CommandXboxController controller;
    private OperatorSubsystem operator;
    private SwerveSubsystem drivebase;
    private TurretSubsystem turret;

    public OperatorInput(int ctlrPort, SwerveSubsystem ss, OperatorSubsystem os, TurretSubsystem ts) {
        controller = new CommandXboxController(ctlrPort);
        operator = os;
        drivebase = ss;
        turret = ts;
    }

    public void init() {
        controller.leftTrigger().whileTrue(operator.indexer.forward().finallyDo(() -> operator.indexer.stop()));
        controller.rightTrigger().whileTrue(operator.intakeAll().finallyDo(() -> operator.indexer.stop()));

        // controller.rightBumper().toggleOnTrue(new AimAtTarget(drivebase, operator, () -> Constants.AimPoints.RED_HUB.value));

        controller.y().whileTrue(operator.climber.climbUp().finallyDo(() -> operator.climber.climbStop()));
        controller.a().whileTrue(operator.climber.climbDown().finallyDo(() -> operator.climber.climbStop()));

        controller.leftBumper().onTrue(Commands.runOnce(()->turret.setAngle(45.0)));
        controller.rightBumper().onTrue(Commands.runOnce(()->turret.setAngle(0.0)));
    }
}
