package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

public class HoldPositionCmd extends Command{
    ShooterSubsystem shooterSubsystem;

    public HoldPositionCmd(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void execute(){
        shooterSubsystem.holdPosition();
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopWrist();
    }
}