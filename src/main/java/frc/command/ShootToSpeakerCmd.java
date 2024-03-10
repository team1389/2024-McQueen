package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.ShooterSubsystem;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class ShootToSpeakerCmd extends Command{
     private ShooterSubsystem shooter;
     private IndexerSubsystem indexer;
     private IntakeSubsystem intake;
     int count;
    public ShootToSpeakerCmd(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake){
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
        if(shooter.getBottomSpeedRPM() > 0){
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

}
