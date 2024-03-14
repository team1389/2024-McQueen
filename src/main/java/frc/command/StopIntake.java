package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class StopIntake extends Command{
    IntakeSubsystem intake;
    IndexerSubsystem indexer;
    public StopIntake(IntakeSubsystem intake, IndexerSubsystem indexer){
        this.intake = intake;
        this.indexer = indexer;
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        indexer.stop();
    }
}
