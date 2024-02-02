package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;
import frc.subsystems.Indexer;

public class Shoot extends Command{
     private Shooter shooter;
     private Indexer indexer;

    public Shoot(Shooter shooter, Indexer indexer){
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(indexer, shooter);
    }

    @Override
    public void execute(){
        indexer.moveToShoot();
        shooter.runShoot();
        
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
        indexer.stop();
    }
}
