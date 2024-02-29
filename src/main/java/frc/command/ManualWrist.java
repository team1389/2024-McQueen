package frc.command;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class ManualWrist extends Command{
    Shooter shooter;
    private final Supplier<Double> power;

    public ManualWrist(Shooter shooter, Supplier<Double> power) {
        this.shooter = shooter;
        this.power = power;
        
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        // shooter.controllerInterrupt = false;
        SmartDashboard.putNumber("Manual Wrist Power", power.get());
        shooter.moveWrist(MathUtil.clamp(power.get(), -0.05, 0.15));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.moveWrist(0);
    }
}
