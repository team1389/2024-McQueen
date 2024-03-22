package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.ShooterSubsystem;

public class ShootPIDCmd extends Command{
     private ShooterSubsystem shooterSubsytem;
     private DriveSubsystem driveSubsystem;
    //  private Intake intakeSubsytem; TBD
     private double shootingRPM;

    public ShootPIDCmd(ShooterSubsystem shooterSubsytem, DriveSubsystem driveSubsystem, double setpoint){
        this.shooterSubsytem = shooterSubsytem;
        this.driveSubsystem = driveSubsystem;
        shootingRPM = setpoint;
      //  SmartDashboard.putNumber("Shooting RPM for Tuning", shootingRPM);
    }


    @Override
    public void execute(){
      //  shootingRPM = SmartDashboard.getNumber("Shooting RPM for Tuning", shootingRPM);
        driveSubsystem.commandAlign = true;
        shooterSubsytem.runShoot(shootingRPM);        
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.commandAlign = false;
        shooterSubsytem.stop();      
    }

    // @Override
    // public boolean isFinished(){
    //     return shooterSubsytem.isAtTargetRPM(shootingRPM);
    // }
}
