package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

//used for tuning hold position command
public class SetPowerCmd extends Command{
    ShooterSubsystem shooter;
    double power;
    public SetPowerCmd(ShooterSubsystem shooter){
        power = .1;
        this.shooter = shooter;
      //  SmartDashboard.putNumber("Power for twunitng", power);
    }

    @Override
    public void execute(){
       // power = SmartDashboard.getNumber("Power for twunitng", power);
        shooter.moveWrist(power);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopWrist();
    }
}
