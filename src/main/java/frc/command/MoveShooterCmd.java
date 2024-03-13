package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.ShooterSubsystem;

public class MoveShooterCmd extends Command{
     private ShooterSubsystem shooter;

    public MoveShooterCmd(ShooterSubsystem shooter){
        this.shooter = shooter;
        // addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.runWristUp();
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopWrist();
    }
}
