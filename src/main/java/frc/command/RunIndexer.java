package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Indexer;

public class RunIndexer extends Command{
    private Indexer indexer;

    public RunIndexer(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute(){
        indexer.moveToShoot();        
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
    }

}
