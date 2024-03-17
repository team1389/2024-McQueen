package frc.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.DriveSubsystem;

public class DriveTowardsRightRedCmd extends Command {
    private final DriveSubsystem drivetrain;
    private final Timer timer;

    public DriveTowardsRightRedCmd(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        timer = new Timer();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // just go forward for one second with no regard for orientation (it's auto, we make the robot start at a known-good pose)
        drivetrain.driveRobotRelative(new ChassisSpeeds(-3, 0, -1.0472));
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
        return timer.hasElapsed(1);
    }
}