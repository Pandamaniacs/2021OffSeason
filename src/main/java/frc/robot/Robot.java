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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController m_driverController = new XboxController(0);
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
  private static final int dtCurrentLimit = 80; //per-motor current limit for the robot; 80A tends to be safe on carpet unless the battery is too weak
  private static final int accelCurrentLimit = 20; //current limiting for the two accelerator rollers
  private static final double bumperTurnRate = 0.5; //smaller numbers mean more aggressive turning

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
    m_feeder = new CANSparkMax(feederID, MotorType.kBrushed);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();
    m_shooter.restoreFactoryDefaults();
    m_accelerator.restoreFactoryDefaults();
    m_feeder.restoreFactoryDefaults();

    // Two of our drivetrain motors act as followers of the other one on the same side, let's make that official.
    // You'll notice we almost never talk about them again; that's intentional.

    m_leftFollow.follow(m_leftMotor);
    m_rightFollow.follow(m_rightMotor);

    // Now, let's set current limiting on our motors.
    m_leftMotor.setSmartCurrentLimit(dtCurrentLimit);
    m_leftFollow.setSmartCurrentLimit(dtCurrentLimit);
    m_rightMotor.setSmartCurrentLimit(dtCurrentLimit);
    m_rightFollow.setSmartCurrentLimit(dtCurrentLimit);
    m_accelerator.setSmartCurrentLimit(accelCurrentLimit);

    // Oh, and we have to create the DifferentialDrive for our drivetrain too. Done.
    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

  }

  @Override
  public void teleopPeriodic() {

    // Let's do quick turns with the bumpers of the driver's controller!

    while (m_driverController.getBumper(Hand.kLeft)) {
      m_leftMotor.set(-m_driverController.getY(Hand.kLeft)*bumperTurnRate);
      m_rightMotor.set(m_driverController.getY(Hand.kLeft));
    }
    
    while (m_driverController.getBumper(Hand.kRight)) {
      m_leftMotor.set(-m_driverController.getY(Hand.kLeft));
      m_rightMotor.set(m_driverController.getY(Hand.kRight)*bumperTurnRate);
    }

    // Oh, we don't have bumpers pressed? Drive with split arcade drive.
    // That means that the Y axis of the left stick moves forward
    // and backward, and the X of the right stick turns left and right.
    while (!m_driverController.getBumper(Hand.kLeft) && !m_driverController.getBumper(Hand.kRight))
    m_myRobot.arcadeDrive(
        m_driverController.getY(Hand.kLeft)*-1, m_driverController.getX(Hand.kRight)*.8);
    
    
    // This next section controlls the ball accelerator, feeder roller, and shooter.
    // You'll notice this is all set off the two Xbox triggers.  It could be better.
    // I hope you'll make this into a couple PID loops down the road.
    m_shooter.set(1.5*(m_driverController.getTriggerAxis(Hand.kLeft)));
    m_accelerator.set(-1*(m_driverController.getTriggerAxis(Hand.kLeft)));
    m_feeder.set(m_driverController.getTriggerAxis(Hand.kRight));
  }
}