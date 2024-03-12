package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class RunOuttakeCmd extends Command{
    private IntakeSubsystem intake;
    private double timer = 0;
    public RunOuttakeCmd(IntakeSubsystem intake){
        this.intake = intake;

        // addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runOuttake();
        timer += 1;
    }

    @Override
    public void end(boolean interrupted){
        timer = 0;
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return (!intake.hitSensor() && timer<25);
    }
}
