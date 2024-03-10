package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class AmpCmd extends SequentialCommandGroup{
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;

    public AmpCmd(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem){
        addCommands(
            new PreAmpCmd(indexerSubsystem, intakeSubsystem), //moves the note into the pre amp position
            new ShootAmpCmd(indexerSubsystem, intakeSubsystem)
        );
    }
}
