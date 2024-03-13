package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

public class ManualWristCmd extends Command{
    ShooterSubsystem shooterSubsystem;
    private final Supplier<Double> power;

    public ManualWristCmd(ShooterSubsystem shooterSubsystem, Supplier<Double> power) {
        this.shooterSubsystem = shooterSubsystem;
        this.power = power;
        shooterSubsystem.controllerInterrupt = true;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void execute() {
        shooterSubsystem.moveWrist(MathUtil.clamp(power.get(), -0.2, 0.2));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.holdPosition();
    }
}
