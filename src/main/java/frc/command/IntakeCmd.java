package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;

public class IntakeCmd extends SequentialCommandGroup{
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    LimelightVisionSubsystem limelight;

    public IntakeCmd(IntakeSubsystem intakeSubsystem, LimelightVisionSubsystem limelight){
        addCommands(
            new RunIntakeCmd(intakeSubsystem, limelight, 4),
            new RunOuttakeCmd(intakeSubsystem, limelight) //moves the note into the pre amp or shoot position
        );
    }
}
