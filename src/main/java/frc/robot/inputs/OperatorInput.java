package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
        if (operator.intake != null) {
            controller.leftTrigger()
                    .whileTrue(operator.intake.forward())
                    .onFalse(operator.intake.stop());
            controller.leftBumper()
                    .whileTrue(operator.intake.reverse())
                    .onFalse(operator.intake.stop());
        }

        if (operator.shooter != null) {
            operator.shooter.setDefaultCommand(new RunCommand(() -> {
                double trig = controller.getRightTriggerAxis();

                if (0 <= trig && trig < 0.20) {
                    Logger.recordOutput("ShooterSpeed", "0%");
                    operator.shooter.set(0).execute();
                } else if (0.20 <= trig && trig < 0.50) {
                    Logger.recordOutput("ShooterSpeed", "35%");
                    operator.shooter.set(-0.35).execute();
                } else if (0.50 <= trig && trig < 0.75) {
                    Logger.recordOutput("ShooterSpeed", "60%");
                    operator.shooter.set(-0.6).execute();
                } else if (0.75 <= trig && trig < 0.90) {
                    Logger.recordOutput("ShooterSpeed", "75%");
                    operator.shooter.set(-0.75).execute();
                } else if (0.90 <= trig && trig < 1.0) {
                    Logger.recordOutput("ShooterSpeed", "100%");
                    operator.shooter.set(-1).execute();
                }
            }, operator.shooter));
        }

        if (operator.intakeSlide != null) {
            controller.povUp()
                    .whileTrue(operator.intakeSlide.extend())
                    .onFalse(operator.intakeSlide.stop());

            controller.povDown()
                    .whileTrue(operator.intakeSlide.retract())
                    .onFalse(operator.intakeSlide.stop());
        }

        if (operator.conveyor != null) {
            controller.y()
                    .whileTrue(operator.conveyor.forward())
                    .onFalse(operator.conveyor.stop());

            controller.a()
                    .whileTrue(operator.conveyor.reverse())
                    .onFalse(operator.conveyor.stop());
        }

        if (operator.indexer != null) {
            controller.b()
                    .whileTrue(operator.indexer.index())
                    .onFalse(operator.indexer.stop());

            controller.x()
                    .whileTrue(operator.indexer.outtake())
                    .onFalse(operator.indexer.stop());
        }
    }
}