package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Intake;

public class RunOuttake extends Command{
    private Intake intake;
    public RunOuttake(Intake intake){
        this.intake = intake;
        // addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runOuttake();
        
    }

    @Override
    public void end(boolean interrupted){

        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return intake.hitSensor();
    }
}
