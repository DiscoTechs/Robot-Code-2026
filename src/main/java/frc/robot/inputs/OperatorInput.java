package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

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
            // climber up and down are inverted
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

        operator.shooter.setDefaultCommand(new RunCommand(() -> {
            double trig = controller.getRightTriggerAxis();

            if (trig < 0.2) {
                Logger.recordOutput("ShooterSpeed", "0");
                operator.shooter.set(0).execute();
            } else {
                if (0.2 <= trig && trig <= 0.5) {
                    Logger.recordOutput("ShooterSpeed", "35");
                    operator.shooter.set(0.35).execute();
                } else if (0.5 <= trig && trig <= 0.75) {
                    Logger.recordOutput("ShooterSpeed", "0.6");
                    operator.shooter.set(0.6).execute();
                } else if (0.76 <= trig && trig <= 1) {
                    Logger.recordOutput("ShooterSpeed", "0.75");
                    operator.shooter.set(0.75).execute();
                }
            }
        }, operator.shooter));
        // controller.leftTrigger().onTrue(Commands.runOnce(() ->
        // operator.shooter.reverse()));
        // controller.leftTrigger().onFalse(Commands.runOnce(() ->
        // operator.shooter.stop()));

        // controller.rightTrigger().onFalse(operator.stopAll());

        controller.rightBumper().onTrue(operator.kicker.forward());
        controller.leftBumper().onTrue(operator.kicker.stop());

        if (operator.intakepivot != null) {
            controller.y()
                    .whileTrue(Commands.runOnce(() -> {
                        System.out.println("FORWARD");
                        operator.intakepivot.forward();
                    }))
                    .onFalse(Commands.runOnce(() -> {
                        System.out.println("STOP");
                        operator.intakepivot.stop();
                    }));

            controller.x()
                    .whileTrue(Commands.runOnce(() -> {
                        System.out.println("BACK");
                        operator.intakepivot.back();
                    }))
                    .onFalse(Commands.runOnce(() -> {
                        System.out.println("STOP");
                        operator.intakepivot.stop();
                    }));
        }

        if (operator.indexer != null && operator.intake != null) {
            controller.a()
                    .whileTrue(Commands.parallel(
                            operator.indexer.forward().asProxy(),
                            operator.intake.forward().asProxy()))
                    .onFalse(Commands.parallel(
                            operator.indexer.stop().asProxy(),
                            operator.intake.stop().asProxy()));

            controller.b()
                    .whileTrue(Commands.parallel(
                            operator.indexer.reverse().asProxy(),
                            operator.intake.reverse().asProxy()))
                    .onFalse(Commands.parallel(
                            operator.indexer.stop().asProxy(),
                            operator.intake.stop().asProxy()));
        }
    }
}