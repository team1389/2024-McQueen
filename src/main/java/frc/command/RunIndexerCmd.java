package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class RunIndexerCmd extends Command{
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;
    boolean isShoot = true;


    public RunIndexerCmd(IndexerSubsystem indexer){
        this.indexer = indexer;
        // addRequirements(indexer);
    }

    public RunIndexerCmd(IndexerSubsystem indexer, boolean isShoot){
        this.indexer = indexer;
        // addRequirements(indexer);
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

    // @Override
    // public boolean isFinished(){
    //     return !intake.hitSensor();
    // }

}
