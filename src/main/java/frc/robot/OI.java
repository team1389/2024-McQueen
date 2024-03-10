package frc.robot;

import java.util.HashMap;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.command.*;
import frc.robot.RobotMap.OIConstants;
import frc.robot.RobotMap.ShooterConstants;
import frc.subsystems.*;
import frc.util.DPadButton;
import frc.util.DPadButton.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkBase.IdleMode;

public class OI {


    private GenericHID driveController;
    private Trigger driveRightBumper, driveLeftBumper;
    private Trigger driveAButton;
    private Trigger driveXButton;
    private Trigger driveBButton;
    private Trigger driveYButton;
    private Trigger driveRightTrigger;
    private Trigger driveLeftTrigger;



    private XboxController manipController;
    private Trigger manipEllipsisButton;
    private Trigger manipMenuButton;
    private Trigger manipFullscreen;
    private Trigger manipStadia;
    private Trigger manipGoogle;


    private Trigger manipRightBumper;
    private Trigger manipLeftBumper;
    private Trigger manipRightTrigger;
    private Trigger manipLeftTrigger;

    private Trigger manipAButton;
    private Trigger manipBButton;
    private Trigger manipXButton;
    private Trigger manipYButton;

    private Trigger manipUp;
    private Trigger manipLeft;
    private Trigger manipDown;
    private Trigger manipRight;
    

    public final DriveSubsystem drivetrainSubsystem = new DriveSubsystem();
    
    public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final LightsSubsystem lightSubsystem = new LightsSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final LimelightVisionSubsystem limeLightVisionSubsystem = new LimelightVisionSubsystem();
    public final PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();

    public OI() {

        initControllers();

        manipAButton.whileTrue(new RunIntakeCmd(intakeSubsystem));

        manipBButton.onTrue(new IntakeCmd(intakeSubsystem));

        manipXButton.onTrue(new ShootCmd(intakeSubsystem,indexerSubsystem,shooterSubsystem));

        manipYButton.onTrue(new AmpCmd(intakeSubsystem,indexerSubsystem));

        manipMenuButton.whileTrue(new RunElevatorDownCmd(elevatorSubsystem));
        manipEllipsisButton.whileTrue(new RunElevatorUpCmd(elevatorSubsystem));

        // manipEllipsisButton.whileTrue(new MoveShooterCmd(shooterSubsystem));
        // manipMenuButton.whileTrue(new MoveShooterDownCmd(shooterSubsystem));


        // manipLeftTrigger.whileTrue(new RunIntakeCmd(intakeSubsystem).andThen(new PreAmpCmd(indexerSubsystem,intakeSubsystem)));

       // manipBButton.whileTrue(new RunIndexer(indexerSubsystem, false));
       // manipYButton.whileTrue(new IndexAndShoot(indexerSubsystem, intakeSubsystem));
        //  manipYButton.whileTrue(new ShootToSpeaker(shooter, indexerSubsystem, intakeSubsystem));
       //  manipYButton.onTrue(new ContinueIntake(intakeSubsystem).alongWith(new RunIndexer(indexerSubsystem, true).alongWith(new Shoot(shooter, intakeSubsystem)))); //TODO
        // manipEllipsisButton.whileTrue(new RunIndexer(indexerSubsystem, true)); // indexerSubsystem to amp
        // manipXButton.onTrue(new RunElevatorUpCmd(elevatorSubsystem).andThen(new RunIntakeCmd(intakeSubsystem)).alongWith(new RunIndexerCmd(indexerSubsystem, false)));
        
        //  manipLeftTrigger.onTrue(new SetElevator(elevatorSubsystem, .605)); //.605 //TODO
        //  manipRightTrigger.onTrue(new SetElevator(elevatorSubsystem, .55)); //.511 //TODO
       // manipLeftTrigger.onTrue(new RunIntake(intakeSubsystem));
        // manipRightTrigger.whileTrue(new RunOuttakeCmd(intakeSubsystem));
        // manipMenuButton.whileTrue(new RunOuttake(intakeSubsystem));
        manipLeftBumper.whileTrue(new ShootPIDCmd(shooterSubsystem, ShooterConstants.kTopRPM).alongWith(new HoldPositionCmd(shooterSubsystem)));
        manipRightBumper.whileTrue(new RunIntakeCmd(intakeSubsystem).alongWith(new RunIndexerCmd(indexerSubsystem, true)));
        // manipYButton.whileTrue(new AlignShooter(shooter, shooter));
       // manipGoogle.onTrue(new InstantCommand(() -> shooter.setTargetAngle(shooter.getWristPos())).alongWith(new InstantCommand(() -> shooter.holdPosition())));
        // manipFullscreen.whileTrue(new SetShoot(shooter));
        
        // manipStadia.whileTrue(new AutoAlign(drivetrainSubsystem, limeLightVisionSubsystem));

        shooterSubsystem.setDefaultCommand(new HoldPositionCmd(shooterSubsystem));
        elevatorSubsystem.setDefaultCommand(new ManualElevatorCmd(elevatorSubsystem, () -> -getManipRightY()));

            drivetrainSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrainSubsystem.drive(
                -MathUtil.applyDeadband(driveController.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driveController.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driveController.getRawAxis(3), OIConstants.kDriveDeadband),
                true, true),
            drivetrainSubsystem));

        // Press A button -> zero gyro headingq
        driveAButton.onTrue(new InstantCommand(() -> drivetrainSubsystem.zeroHeading()));

        // Press X button -> set X to not slide
        driveXButton.onTrue(new InstantCommand(() -> drivetrainSubsystem.setX()));

        driveYButton.onTrue(new InstantCommand(() -> {lightSubsystem.isRainbowing = true;}));

        NamedCommands.registerCommand("Shoot", new AutoShootPIDCmd(shooterSubsystem, ShooterConstants.kTopRPM));
        NamedCommands.registerCommand("IndexerToShooter", new RunIndexerCmd(indexerSubsystem, true));
        NamedCommands.registerCommand("IndexerToAmp", new RunIndexerCmd(indexerSubsystem, false));
        NamedCommands.registerCommand("RunIntake", new RunIntakeCmd(intakeSubsystem));
        
        getAutonomousCommand();

        // Create a path following command using AutoBuilder. This will also trigger event markers.
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);

        manipAButton = new JoystickButton(manipController,1);
        manipBButton = new JoystickButton(manipController, 2); 
        manipXButton = new JoystickButton(manipController, 3);
        manipYButton = new JoystickButton(manipController, 4);

        manipRightBumper = new JoystickButton(manipController, 6);
        manipRightTrigger = new JoystickButton(manipController, 12);
        manipLeftTrigger = new JoystickButton(manipController, 13);
        manipLeftBumper = new JoystickButton(manipController, 5);

        manipMenuButton = new JoystickButton(manipController, 10);
        manipFullscreen = new JoystickButton(manipController, 15);
        manipGoogle = new JoystickButton(manipController, 14);
        manipEllipsisButton = new JoystickButton(manipController, 9);

        driveRightBumper = new JoystickButton(driveController, 6);
        driveRightTrigger = new JoystickButton(driveController, 12);
        driveLeftTrigger = new JoystickButton(driveController, 13);
        driveLeftBumper = new JoystickButton(driveController, 5);


        driveAButton = new JoystickButton(driveController, 1);
        driveBButton = new JoystickButton(driveController, 2); 
        driveXButton = new JoystickButton(driveController, 3); 
        driveYButton = new JoystickButton(driveController, 4); 


    }

    private double getManipLeftY() {
        return manipController.getRawAxis(1);
    }
    
    private double getManipRightY() {
        return manipController.getRawAxis(4);
    }

    private double getDriveLeftX() {
        return driveController.getRawAxis(0);
    }
    
    private double getDriveLeftY() {
        return -driveController.getRawAxis(1);
    }
    
    private double getDriveRightX() {
        return -driveController.getRawAxis(3); 
    }
    
    private double getDriveRightY() {
        return driveController.getRawAxis(4); 
    }
    
    private boolean getDriveLeftBumper() {
        return !driveController.getRawButton(5);
    }
    
    private boolean getDriveRightBumper() {
        return !driveController.getRawButton(6);
    }

    public Command getAutonomousCommand() {
    return new PathPlannerAuto("Top Drive Shoot");
  }


}