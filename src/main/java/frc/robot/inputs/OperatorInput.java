package frc.robot.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.OperatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class OperatorInput {
    public static CommandXboxController controller;
    private OperatorSubsystem operator;
    private SwerveSubsystem drivebase;

    public OperatorInput(int ctlrPort, SwerveSubsystem ss, OperatorSubsystem os) {
        controller = new CommandXboxController(ctlrPort);
        drivebase = ss;
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
            controller.povUp()
                    .whileTrue(operator.climber.climbUp())
                    .onFalse(operator.climber.stop());

            controller.povDown()
                    .whileTrue(operator.climber.climbDown())
                    .onFalse(operator.climber.stop());
        }

        if (operator.turret != null) {
            controller.leftStick().onChange(Commands.runOnce(() -> {
                // TODO: PLEASE JAKE ADD SOME FUCKING SAFEGAURDS

                double leftAxis = controller.getLeftX();
                if (-0.1 < leftAxis && leftAxis < 0.1) {
                    operator.turret.set(0);
                } else if (leftAxis > 0) {
                    // GO RIGHT
                    operator.turret.set(0.5);
                } else if (leftAxis < 0) {
                    // GO LEFT
                    operator.turret.set(-0.5);
                }
            }));
        }

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