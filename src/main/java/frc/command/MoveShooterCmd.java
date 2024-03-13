package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.ShooterSubsystem;

public class MoveShooterCmd extends Command{
     private ShooterSubsystem shooterSubsystem;

    public MoveShooterCmd(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        // addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.runWristUp();
        
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopWrist();
    }
}
