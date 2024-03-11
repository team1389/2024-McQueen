package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.ShooterSubsystem;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class PreShootCmd extends Command{
     private ShooterSubsystem shooter;
     private IndexerSubsystem indexer;
     private IntakeSubsystem intake;
     int timer = 0;
    public PreShootCmd(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter){
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;

        // addRequirements(intake, indexer, shooter);
    }

    @Override
    public void execute(){
        indexer.moveToShoot();
        intake.runIntake();
        // timer += 1;
        //addCommand(new WaitCommand(5));        
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
        intake.stop();
        shooter.stop();
        // timer = 0;
    }

    @Override
    public boolean isFinished(){
        // return (timer>20);
        return !(intake.hitSensor());
    }

}
