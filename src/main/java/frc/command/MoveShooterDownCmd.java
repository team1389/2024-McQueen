package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.ShooterSubsystem;

public class MoveShooterDownCmd extends Command{
     private ShooterSubsystem shooterSubsystem;

    public MoveShooterDownCmd(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        // addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.runWristDown();
        
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopWrist();
    }
}
