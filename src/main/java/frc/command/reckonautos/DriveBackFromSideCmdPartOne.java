package frc.command.reckonautos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.DriveSubsystem;

public class DriveBackFromSideCmdPartOne extends Command {
    private final DriveSubsystem drivetrain;
    private final Timer timer;

    public DriveBackFromSideCmdPartOne(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        timer = new Timer();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // just go forward for one second with no regard for orientation (it's auto, we make the robot start at a known-good pose)
        drivetrain.driveRobotRelative(new ChassisSpeeds(2, 0, 0));
        timer.start();
    }

    @Override
    public void execute() {
        // no need to constantly call driveRobotRelative
    }

    @Override
    public void end(boolean interrupted) {
        // stop the robot when ended
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(.25);
    }
}