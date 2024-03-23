package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.util.LimelightHelpers;

public class InfiniteIntake extends Command{
    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;

    public InfiniteIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
    }

    @Override
    public void execute(){
        intakeSubsystem.runIntake();
        indexerSubsystem.moveToShoot();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
        indexerSubsystem.stop();
    }
}
