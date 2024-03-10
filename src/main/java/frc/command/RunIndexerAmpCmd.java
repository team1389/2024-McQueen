package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;

public class RunIndexerAmpCmd extends Command{
    private IndexerSubsystem indexer;


    public RunIndexerAmpCmd(IndexerSubsystem indexer){
        this.indexer = indexer;
        // addRequirements(indexer);
    }

    public RunIndexerAmpCmd(IndexerSubsystem indexer, boolean isShoot){
        this.indexer = indexer;
        // addRequirements(indexer);
    }

    @Override
    public void execute(){
        indexer.moveToAmp();
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
    }

}