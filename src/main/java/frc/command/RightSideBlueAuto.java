package frc.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;

public class RightSideBlueAuto extends SequentialCommandGroup{
    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private LimelightVisionSubsystem limelight;
    private DriveSubsystem driveSubsystem;

    public RightSideBlueAuto(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem limelight, DriveSubsystem driveSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.limelight = limelight;
        addCommands(
            new AutoSetWristCmd(shooterSubsystem, .95, limelight),
            new AutoShootPIDCmd(shooterSubsystem, limelight.rpmTableForShoot(),limelight),
            new PreShootCmd(indexerSubsystem,intakeSubsystem, shooterSubsystem),
            new DriveBackFromSideCmdPartOne(driveSubsystem),
            new BlueRightRotateAwayAuto(driveSubsystem),
            Commands.parallel(
                new DriveBackFromSideCmdPartTwo(driveSubsystem),
                new IntakeCmd(intakeSubsystem)
            ),
            new DriveTowardSideCmdPartOne(driveSubsystem),
            new BlueRightRotateTowardAuto(driveSubsystem),
            new DriveTowardSideCmdPartTwo(driveSubsystem),
            new AutoShootPIDCmd(shooterSubsystem, limelight.rpmTableForShoot(),limelight),
            new PreShootCmd(indexerSubsystem,intakeSubsystem, shooterSubsystem)
        );
    }
}
