package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class PreAmpCmd extends Command{
    private IndexerSubsystem indexerSubsystem;
    private IntakeSubsystem intakeSubsystem;


    public PreAmpCmd(IndexerSubsystem indexerSubsystem){
        this.indexerSubsystem = indexerSubsystem;
        // addRequirements(indexerSubsystem);
    }

    public PreAmpCmd(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem){
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        // addRequirements(indexerSubsystem);
    }

    @Override
    public void execute(){
        indexerSubsystem.moveToAmp();
        intakeSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted){
        indexerSubsystem.stop();
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return !intakeSubsystem.hitSensor();
    }

}