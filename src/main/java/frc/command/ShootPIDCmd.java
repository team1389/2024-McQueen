package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.ShooterConstants;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.ShooterSubsystem;

public class ShootPIDCmd extends Command{
     private ShooterSubsystem shooterSubsytem;
    //  private Intake intakeSubsytem; TBD
     private double shootingRPM;

    public ShootPIDCmd(ShooterSubsystem shooterSubsytem, double setpoint){
        this.shooterSubsytem = shooterSubsytem;
        shootingRPM = setpoint;
        SmartDashboard.putNumber("Shooting RPM for Tuning", shootingRPM);
    }


    @Override
    public void execute(){
        shootingRPM = SmartDashboard.getNumber("Shooting RPM for Tuning", shootingRPM);
        shooterSubsytem.runShoot(shootingRPM);        
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsytem.stop();      
    }

    @Override
    public boolean isFinished(){
        return shooterSubsytem.isAtTargetRPM(shootingRPM);
    }
}
