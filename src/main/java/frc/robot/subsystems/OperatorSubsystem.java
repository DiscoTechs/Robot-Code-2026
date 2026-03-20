package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorSubsystem extends SubsystemBase {
    public final IntakeSlideSubsystem intakeSlide;
    public final ConveyorSubsystem conveyor;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;

    public AngularVelocity targetShooterSpeed = RPM.of(6000);
    public final Trigger isShooterAtSpeed;

    public OperatorSubsystem(IndexerSubsystem ind, ShooterSubsystem shoot, IntakeSlideSubsystem intSlide, IntakeSubsystem intake, ConveyorSubsystem convey) {
        this.intakeSlide = intSlide;
        this.conveyor = convey;
        this.indexer = ind;
        this.shooter = shoot;
        this.intake = intake;

        this.isShooterAtSpeed = new Trigger(() -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < RPM.of(100).in(RPM));
    }

    @Override
    public void periodic() {
        // System.out.println("Shooter: " + isShooterAtSpeed.getAsBoolean() + " (" + Math.round(shooter.getSpeed().in(RPM))
        //         + "/" + Math.round(targetShooterSpeed.in(RPM)) + " RPM)");
    }
}
