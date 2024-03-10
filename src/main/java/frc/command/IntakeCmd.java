package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class IntakeCmd extends SequentialCommandGroup{
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;

    public IntakeCmd(IntakeSubsystem intakeSubsystem){
        addCommands(
            new RunIntakeCmd(intakeSubsystem),
            new RunOuttakeCmd(intakeSubsystem) //moves the note into the pre amp position
        );
    }
}
