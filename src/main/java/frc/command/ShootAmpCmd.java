package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class ShootAmpCmd extends Command{
    private IndexerSubsystem indexer;
    private IntakeSubsystem intakeSubsystem;
    private double timer = 0;

    public ShootAmpCmd(IndexerSubsystem indexer, IntakeSubsystem intake){
        this.indexer = indexer;
        this.intakeSubsystem = intake;
        // addRequirements(indexer);
    }

    @Override
    public void execute(){
        intakeSubsystem.runIntake();
        indexer.moveToShoot();
        timer += 1;
    }

    @Override
    public void end(boolean interrupted){
        indexer.stop();
        intakeSubsystem.stop();
        timer = 0;
    }

    @Override
    public boolean isFinished(){
        return (timer>20);
    }

}