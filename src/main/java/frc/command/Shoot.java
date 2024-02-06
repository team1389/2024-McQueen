package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;
import frc.subsystems.Indexer;
import frc.subsystems.Intake;

public class Shoot extends Command{
     private Shooter shooter;
     private Indexer indexer;
     private Intake intake;

    public Shoot(Shooter shooter, Indexer indexer, Intake intake){
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        addRequirements(intake, indexer, shooter);
    }

    @Override
    public void execute(){
        intake.runIntake();
        indexer.moveToShoot();
        shooter.runShoot();
        
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        shooter.stop();
        indexer.stop();
    }
}
