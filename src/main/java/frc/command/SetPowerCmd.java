package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

//used for tuning hold position command
public class SetPowerCmd extends Command{
    ShooterSubsystem shooter;
    double power;
    public SetPowerCmd(ShooterSubsystem shooter,double power){
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void execute(){
        shooter.moveWrist(power);
    }

    @Override
    public void end(boolean interrupted){
        shooter.holdPosition();
    }
}
