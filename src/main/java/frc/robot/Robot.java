/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController m_driverController = new XboxController(0);
  private XboxController m_gunnerController = new XboxController(1);
  private static final int leftDeviceID = 1; 
  private static final int leftFollowID = 2;
  private static final int rightDeviceID = 3;
  private static final int rightFollowID = 4;
  private static final int shooterID = 21;
  private static final int acceleratorID = 20;
  private static final int feederID = 10;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftFollow;
  private CANSparkMax m_rightFollow;
  private CANSparkMax m_shooter;
  private CANSparkMax m_accelerator;
  private CANSparkMax m_feeder;
  private final Timer m_timer = new Timer();

    // Here is where you set the current limits for drivetrain and for the accelerator rollers.
    // Consider whether the motor can sustain that current for however long it may be jammed up.
    // Drivetrains can be more aggressive than conveyors; it's more obvious when the driver needs to back off.
    // Consult locked rotor or constant current test data on motors.vex.com.

  private static final int dtCurrentLimit = 80; // per-motor current limit for the drivetrain
  private static final int accelCurrentLimit = 20; // current limiting for the two accelerator rollers; 20A is super safe for a NEO 550.

  // And climbers.  These use the classic SPARK motor controllers.
  Spark leftClimber = new Spark(0);
  Spark rightClimber = new Spark(1);

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    m_leftFollow = new CANSparkMax(leftFollowID, MotorType.kBrushless);
    m_rightFollow = new CANSparkMax(rightFollowID, MotorType.kBrushless);
    m_shooter = new CANSparkMax(shooterID, MotorType.kBrushless);
    m_accelerator = new CANSparkMax(acceleratorID, MotorType.kBrushless);
    // The feeder runs a 775pro motor unlike the other SPARK MAX controllers that drive NEO-series
    // motors.  So it gets set up as Brushed, not Brushless.
    m_feeder = new CANSparkMax(feederID, MotorType.kBrushed);

    /**
     * The RestoreFactoryDefaults method ensures settings are cleared. This is important when you
     * replace a SPARK MAX; by setting settings here, we don't have to remember all the settings the
     * dead one had. Everything is in the code, just get the CAN ID right and it'll do the rest.
     */
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();
    m_shooter.restoreFactoryDefaults();
    m_accelerator.restoreFactoryDefaults();
    m_feeder.restoreFactoryDefaults();

    // Two of our drivetrain motors act as followers of the other one on the same side, let's make it official.
    // You'll notice we almost never talk about them again; that's intentional.

    m_leftFollow.follow(m_leftMotor);
    m_rightFollow.follow(m_rightMotor);

    // Now, let's set current limiting on our motors so the robot doesn't brown out when launching.

    m_leftMotor.setSmartCurrentLimit(dtCurrentLimit);
    m_leftFollow.setSmartCurrentLimit(dtCurrentLimit);
    m_rightMotor.setSmartCurrentLimit(dtCurrentLimit);
    m_rightFollow.setSmartCurrentLimit(dtCurrentLimit);
    m_accelerator.setSmartCurrentLimit(accelCurrentLimit);

    // Oh, and we have to create the DifferentialDrive for our drivetrain too. Done.
    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);


  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_myRobot.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_myRobot.stopMotor(); // stop robot
    }
  }

  @Override
  public void teleopPeriodic() {
    // Now let's drive with split arcade drive.
    // That means that the Y axis of the left stick moves forward
    // and backward, and the X of the right stick turns left and right.
    // Left stick is inverted to get the direction correct.
    // Right stick has a multiplier to make it less twitchy.
    
    m_myRobot.arcadeDrive(
    m_driverController.getY(Hand.kLeft)*-1, m_driverController.getX(Hand.kRight)*.8);
    
    // This next section controlls the ball accelerator, feeder roller, and shooter.
    // You'll notice this is all set off the two Xbox triggers.  It could be better.
    // I hope you'll make this into a couple PID loops down the road.

    m_shooter.set(1.0*(m_gunnerController.getTriggerAxis(Hand.kLeft)));
    m_accelerator.set(-.8*(m_gunnerController.getTriggerAxis(Hand.kLeft)));
    m_feeder.set(m_gunnerController.getTriggerAxis(Hand.kRight));

// Let's do bumper shooting action!

    if (m_gunnerController.getBumper(Hand.kLeft)) {
      m_shooter.set(.75);
      m_accelerator.set(-.6);
    } else {
      m_shooter.set(0);
      m_accelerator.set(0);
    }

    if (m_gunnerController.getBumper(Hand.kRight)) {
      m_feeder.set(0.5);
    } else {
      m_feeder.set(0);
    }

    // Climber action goes here.  Holding both driver bumpers runs the motors.

    if (m_driverController.getBumper(Hand.kLeft) && m_driverController.getBumper(Hand.kRight)) {

    // Lori seems happy with these settings, but you can bump them up. Probably should. Verify ratchet direction before securing them.

      leftClimber.set(0.75);
      rightClimber.set(0.75);
    } else {
      leftClimber.set(0);
      rightClimber.set(0);
    }
  }
}