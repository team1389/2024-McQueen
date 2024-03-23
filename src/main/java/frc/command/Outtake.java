package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;

public class Outtake extends Command{
    private IntakeSubsystem intakeSubsystem;
    private LimelightVisionSubsystem limelight;
    // private double timer = 0;
    public Outtake(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.limelight = limelight;

        // addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.runOuttake();
        // timer += 1;
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }
}
