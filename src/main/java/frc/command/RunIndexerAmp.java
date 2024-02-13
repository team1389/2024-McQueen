package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Indexer;

public class RunIndexerAmp extends Command{
    private Indexer indexer;
    boolean isShoot = true;


    public RunIndexerAmp(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }

    public RunIndexerAmp(Indexer indexer, boolean isShoot){
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute(){
        if(isShoot){
            indexer.moveToShoot();
        } else{
            indexer.moveToAmp();
        }
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
    }

}
