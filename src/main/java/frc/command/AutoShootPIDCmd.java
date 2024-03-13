package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.ShooterConstants;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;

public class AutoShootPIDCmd extends Command{
     private ShooterSubsystem shooterSubsytem;
     private LimelightVisionSubsystem limelightSubsystem;
    //  private Intake intakeSubsytem; TBD
     private double shootingRPM;

    public AutoShootPIDCmd(ShooterSubsystem shooterSubsytem, double rpm, LimelightVisionSubsystem limelightSubsystem){
        this.shooterSubsytem = shooterSubsytem;
        this.limelightSubsystem = limelightSubsystem;
        // this.intakeSubsytem = intakeSubsytem;
        SmartDashboard.putNumber("Shooting RPM for Tuning", shootingRPM);
        // addRequirements(shooterSubsystem);
    }


    @Override
    public void execute(){
        // shootingRPM = SmartDashboard.getNumber("Shooting RPM for Tuning", shootingRPM);
        shootingRPM = limelightSubsystem.rpmTableForShoot();
        // shooterSubsytem.runShoot(shootingRPM);
        shooterSubsytem.runShoot(shootingRPM);
    }

    @Override
    public void end(boolean interrupted){
        // shooterSubsytem.runShoot(shootingRPM);
        shooterSubsytem.stop();
    }

    @Override
    public boolean isFinished(){
        return shooterSubsytem.isAtTargetRPM(shootingRPM);
    }
}
