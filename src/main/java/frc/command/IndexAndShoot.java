package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.Indexer;
import frc.subsystems.Intake;

public class IndexAndShoot extends Command{
     private Indexer indexer;
     private Intake intake;

    public IndexAndShoot(Indexer indexer, Intake intake){
        this.indexer = indexer;
        this.intake = intake;
        addRequirements(intake, indexer);
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
