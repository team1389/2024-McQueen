package frc.robot;

import java.util.HashMap;

import frc.command.*;
import frc.subsystems.*;
import frc.util.DPadButton;
import frc.util.DPadButton.Direction;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    
    public final Drivetrain drivetrain = new Drivetrain();
    
    public final Indexer indexer = new Indexer();

    public final Intake intake = new Intake();
    public final Lights light = new Lights();
    public final Shooter shooter = new Shooter();
    public final Elevator elevator = new Elevator();

    public OI() {
        
        initControllers();
        manipAButton.whileTrue(new RunIntake(intake).andThen(new InstantCommand(() -> light.setColor(0, 128, 255))));
        manipBButton.whileTrue(new RunIndexerAmp(indexer));
        manipFullscreen.whileTrue(new Shoot(shooter));
         manipYButton.whileTrue(new ShootToSpeaker(shooter, indexer, intake));
        // manipEllipsisButton.whileTrue(new RunIndexer(indexer, true)); // indexer to amp
        manipXButton.whileTrue(new RunElevatorUp(elevator).andThen(new RunIntake(intake)).alongWith(new RunIndexer(indexer, true)));
        manipLeftTrigger.onTrue(new RunElevatorDown(elevator));
        manipRightTrigger.onTrue(new RunElevatorUp(elevator));
        manipMenuButton.whileTrue(new RunOuttake(intake));
        manipLeftBumper.whileTrue(new MoveShooter(shooter));
        manipRightBumper.whileTrue(new MoveShooterDown(shooter));

        // Cool new way to make a drive command by passing in Suppliers for the
        // joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
                drivetrain,
                () -> getDriveLeftY(),
                () -> getDriveLeftX(),
                () -> getDriveRightX(),
                () -> getDriveRightY(),
                () -> getDriveLeftBumper(), // By default be in field oriented
                () -> !getDriveRightBumper(), // Slow function
                () -> driveXButton.getAsBoolean(), // Hold x position
                () -> driveRightTrigger.getAsBoolean(),
                () -> driveController.getRawAxis(5)) // flip
        );

        shooter.setDefaultCommand(new ManualWrist(shooter, getManipLeftY()));
        elevator.setDefaultCommand(new ManualElevator(elevator, getManipRightY()));


        // Press A button -> zero gyro heading
        driveAButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

        driveYButton.onTrue(new InstantCommand(() -> {light.isRainbowing = true;}));


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
        return -manipController.getRawAxis(1);
    }
    
    private double getManipRightY() {
        return -manipController.getRawAxis(4);
    }

    private double getDriveLeftX() {
        return driveController.getRawAxis(0);
    }
    
    private double getDriveLeftY() {
        return -driveController.getRawAxis(1);
    }
    
    private double getDriveRightX() {
        return driveController.getRawAxis(3); 
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


}