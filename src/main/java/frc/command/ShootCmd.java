package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.ShooterSubsystem;


public class ShootCmd extends SequentialCommandGroup{
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;

    public ShootCmd(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem){
        addCommands(
            new SetWristCmd(shooterSubsystem,0.85),
            Commands.parallel(
                new HoldPositionCmd(shooterSubsystem),
                Commands.sequence(
                    new AutoShootPIDCmd(shooterSubsystem, 3000),
                    new PreShootCmd(indexerSubsystem,intakeSubsystem, shooterSubsystem)
                )
            )
        );
    }
}