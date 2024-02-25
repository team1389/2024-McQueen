package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class HoldPosition extends Command {
    Shooter shooter;

    public HoldPosition(Shooter shooter) {
        this.shooter = shooter;
        shooter.setTargetAngle(shooter.getWristPos());
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.holdPosition();
    }

    // public void initialize() {
    //     shooter.controllerInterrupt = false;
        
    //     shooter.moveWrist(shooter.getWristPos());
    // }


    public boolean isFinished() {
        return true;
    }
}
