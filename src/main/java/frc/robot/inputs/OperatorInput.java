package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import static edu.wpi.first.units.Units.Degrees;
import frc.robot.subsystems.OperatorSubsystem;

public class OperatorInput {
    public static CommandXboxController controller;
    private OperatorSubsystem operator;

    public OperatorInput(int ctlrPort, OperatorSubsystem os) {
        controller = new CommandXboxController(ctlrPort);
        operator = os;
    }

    public void init() {
        if (operator.indexer != null) {
            controller.leftTrigger()
                    .whileTrue(operator.indexer.forward())
                    .onFalse(operator.indexer.stop());

            controller.rightTrigger()
                    .whileTrue(operator.intakeAll())
                    .onFalse(operator.indexer.stop());
        }

        if (operator.climber != null) {
            controller.povDown()
                    .whileTrue(operator.climber.climbDown())
                    .onFalse(operator.climber.stop());
//climber up and down are inverted 
            controller.povUp()
                    .whileTrue(operator.climber.climbUp())
                    .onFalse(operator.climber.stop());
        }

        if (operator.turret != null) {
            operator.turret.setDefaultCommand(new RunCommand(() -> {
                double leftAxis = controller.getLeftX();
                if (Math.abs(leftAxis) < 0.1) {
                    operator.turret.set(0).execute();
                } else {
                    operator.turret.set(leftAxis * 0.5).execute();
                }
            }, operator.turret));
        }

        controller.rightTrigger().onTrue(operator.intakeAll());
        controller.rightTrigger().onFalse(operator.stopAll());

        // controller.leftTrigger().onTrue(Commands.runOnce(() -> operator.shooter.reverse()));
        // controller.leftTrigger().onFalse(Commands.runOnce(() -> operator.shooter.stop()));

        if (operator.intakepivot != null) {
            controller.y()
                    .whileTrue(operator.intakepivot.setAngle(Degrees.of(45)))
                    .onFalse(operator.intakepivot.stop());
                   

            controller.x()
                    .whileTrue(operator.intakepivot.setAngle(Degrees.of(-45)))
                    .onFalse(operator.intakepivot.stop());
        }

        if (operator.intake != null) {
            controller.a()
                    .whileTrue(operator.intake.forward())
                    .onFalse(operator.intake.stop());
        }
    }
}