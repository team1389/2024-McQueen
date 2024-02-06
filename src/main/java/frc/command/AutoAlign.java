package frc.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;
import frc.subsystems.LimelightVision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.util.LimelightHelpers;

public class AutoAlign extends Command{

    private final Drivetrain drivetrain;
    private final LimelightVision limeLightVision;
    double tx;

     public AutoAlign(Drivetrain drivetrain, LimelightVision limeLightVision) {
        this.drivetrain = drivetrain;
        this.limeLightVision = limeLightVision;

        addRequirements(drivetrain, limeLightVision);
    }


    @Override
    public void execute() {        

        tx = LimelightHelpers.getTX("");

        double speed = 0.1;

        //math
        double targetAngle = 0;
        double rotAngle = tx;

       // rotAngle = tx;

        SmartDashboard.putNumber("Rotation Angle", Math.toDegrees(rotAngle));
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));

        if (Math.abs(rotAngle) < 0.5) {
            drivetrain.stopModules();
            //rotAngle*speed
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,
                    0, speed * rotAngle);
            SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    
}
