package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class IndexAndShootCmd extends Command{
     private IndexerSubsystem indexer;
     private IntakeSubsystem intake;

    public IndexAndShootCmd(IndexerSubsystem indexer, IntakeSubsystem intake){
        this.indexer = indexer;
        this.intake = intake;
        // addRequirements(intake, indexer);
    }

    @Override
    public void execute(){
        intake.runIntake();
        indexer.moveToShoot();
        
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        indexer.stop();
    }
}
