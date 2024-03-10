package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class HoldPositionCmd extends Command{
    Shooter shooter;

    public HoldPositionCmd(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void execute(){
        shooter.holdPosition();
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopWrist();
    }
}