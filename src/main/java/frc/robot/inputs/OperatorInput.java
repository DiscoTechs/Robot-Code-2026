package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.OperatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.commands.AimAtTarget;

public class OperatorInput {
    public static CommandXboxController controller;
    private OperatorSubsystem operator;
    private SwerveSubsystem drivebase;

    public OperatorInput(int ctlrPort, SwerveSubsystem ss, OperatorSubsystem os) {
        controller = new CommandXboxController(ctlrPort);
        operator = os;
        drivebase = ss;
    }

    public void init() {
        controller.leftTrigger().whileTrue(operator.indexerOuttake().finallyDo(() -> operator.indexerStop()));
        controller.rightTrigger().whileTrue(operator.intakeAll().finallyDo(() -> operator.indexerStop()));

        // controller.rightBumper().toggleOnTrue(new AimAtTarget(drivebase, operator, () -> Constants.AimPoints.RED_HUB.value));

        controller.y().whileTrue(operator.climbUp());
        controller.a().whileTrue(operator.climbDown());
    }
}
