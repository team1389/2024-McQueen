package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class ShootAmpCmd extends Command{
    private IndexerSubsystem indexerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double timer = 0;

    public ShootAmpCmd(IndexerSubsystem indexerSubsystem, IntakeSubsystem intake){
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intake;
        // addRequirements(indexerSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.runIntake();
        indexerSubsystem.moveToShoot();
        timer += 1;
    }

    @Override
    public void end(boolean interrupted){
        indexerSubsystem.stop();
        intakeSubsystem.stop();
        timer = 0;
    }

    @Override
    public boolean isFinished(){
        return (timer>25);
        // return !intakeSubsystem.hitSensor();
    }

}