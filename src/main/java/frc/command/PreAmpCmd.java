package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class PreAmpCmd extends Command{
    private IndexerSubsystem indexer;
    private IntakeSubsystem intakeSubsystem;


    public PreAmpCmd(IndexerSubsystem indexer){
        this.indexer = indexer;
        // addRequirements(indexer);
    }

    public PreAmpCmd(IndexerSubsystem indexer, IntakeSubsystem intake){
        this.indexer = indexer;
        this.intakeSubsystem = intake;
        // addRequirements(indexer);
    }

    @Override
    public void execute(){
        indexer.moveToAmp();
        intakeSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return !intakeSubsystem.hitSensor();
    }

}