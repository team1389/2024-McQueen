package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.Shooter;
import frc.subsystems.Indexer;
import frc.subsystems.Intake;

public class ShootToSpeaker extends Command{
     private Shooter shooter;
     private Indexer indexer;
     private Intake intake;
     int count;
    public ShootToSpeaker(Shooter shooter, Indexer indexer, Intake intake){
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        count = 0;
        // addRequirements(intake, indexer, shooter);
    }

    @Override
    public void execute(){
        shooter.runShoot();
        count++;
        if(count > 40){
            intake.runIntake();
            indexer.moveToShoot();
        }
        //addCommand(new WaitCommand(5));        
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
        shooter.stop();
        indexer.stop();
    }

    // @Override
    // public boolean isFinished(){
    //     return 

    // }
}
