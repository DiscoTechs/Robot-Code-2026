package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorSubsystem extends SubsystemBase {
    public final ClimberSubsystem climber;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;
    public final TurretSubsystem turret;
    public final IntakePivotSubsystem intakepivot;
    public final IntakeSubsystem intake;
    public final KickerSubsystem kicker;

    public AngularVelocity targetShooterSpeed = RPM.of(0);
    public final Trigger isShooterAtSpeed;

    public OperatorSubsystem(ClimberSubsystem climb, IndexerSubsystem ind, ShooterSubsystem shoot,
            TurretSubsystem turr, IntakePivotSubsystem intPiv, IntakeSubsystem intake, KickerSubsystem kick) {
        this.climber = climb;
        this.shooter = shoot;
        this.indexer = ind;
        this.turret = turr;
        this.intakepivot = intPiv;
        this.intake = intake;
        this.kicker = kick;

        this.isShooterAtSpeed = new Trigger(
                () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < RPM.of(100).in(RPM));
    }

    public Command intakeAll() {
        return Commands.parallel(
                (new WaitCommand(0.5).andThen(indexer::forward)).asProxy(),
                shooter.forward().asProxy());
    }

    public Command outtakeAll() {
        return Commands.parallel(
                indexer.reverse().asProxy(),
                shooter.reverse().asProxy());
    }

    public Command stopAll() {
        return Commands.parallel(
                indexer.stop().asProxy(),
                shooter.stop().asProxy());
    }

    @Override
    public void periodic() {
        System.out.println("Shooter: " + isShooterAtSpeed.getAsBoolean() + " (" + Math.round(shooter.getSpeed().in(RPM)) + "/" + Math.round(targetShooterSpeed.in(RPM)) + " RPM)");
    }
}
