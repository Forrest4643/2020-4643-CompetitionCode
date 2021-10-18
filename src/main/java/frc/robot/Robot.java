/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//this is a test 

//this is a test for "testbranch"

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.SpeedControllerGroup; //for grouping speed controllers
import com.revrobotics.CANSparkMax; //for spark max speed controllers
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //for defining type of motor
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //for controlling robot with diff drive
import edu.wpi.first.wpilibj.GenericHID.Hand; //for HID hand

import com.ctre.phoenix.motorcontrol.ControlMode; //for talonSRX control modes
import com.ctre.phoenix.motorcontrol.can.TalonSRX; //for using talonSRX over can 
import com.kauailabs.navx.frc.AHRS; //for navX
import edu.wpi.first.wpilibj.SPI; //for navX
import edu.wpi.first.wpilibj.Joystick; //for navX
import edu.wpi.first.wpilibj.Timer; //for making delays
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //allows us to communicate with smart dashboard
import edu.wpi.first.networktables.NetworkTableInstance; //for network devices
import edu.wpi.first.wpilibj.AnalogGyro; //for analog gyros
import edu.wpi.first.wpilibj.AnalogInput; //for analog imputs
import edu.wpi.first.wpilibj.DigitalInput; //for digital imputs
import edu.wpi.first.wpilibj.XboxController; //for xbox compatible controllers
import edu.wpi.first.wpilibj.buttons.JoystickButton; //for buttons on joystick controllers
import com.ctre.phoenix.motorcontrol.*; //for talonSRX motorcontrol
import edu.wpi.first.wpilibj.Compressor; //for PCM compressor control 
import edu.wpi.first.wpilibj.DoubleSolenoid; //for PCM solonoid control
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;  //for usb camera use
import edu.wpi.first.cameraserver.CameraServer; //for network camer use
import edu.wpi.first.wpilibj.DoubleSolenoid.Value; //for foreward - reverse solonoid control
import edu.wpi.first.wpilibj.Timer;
import java.lang.*;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;
   // defines the sonar sensor
   private static final AnalogInput sonarsensor1 = new AnalogInput(1);
   // this is the multiplier to get the distace from the volts
   private static final double Voltmult = 100.0 * 30.48;
   //defines the sonar sensor in the shooter
   private static final AnalogInput shooterSonar = new AnalogInput(2);
 
   // gives us the voltage
   public static double getVoltage() {
     return sonarsensor1.getVoltage();
   }
 
   // gives us the distance in Feet
   public static double Feet() {
     return getVoltage() * Voltmult;
   }
  //limelight steering kP
  private double SteerkP = 0.05;
  //limelight steering min_commamd
  private double steerMin_command = 0.0;
  //limelight speed kP
  private double SpeedkP = 0.1;
  //limelight speed min_command
  private double  speedMin_command = 0.0;
  
  //manip buttons = com port 2
  private final XboxController buttonBoard = new XboxController(2);
  //drive steering wheel and pedals = com port 1
  private final XboxController SteeringWheel = new XboxController(1);

  //defining solonoids for gearboxes
  DoubleSolenoid Shifter = new DoubleSolenoid(7, 0);
  //defining compressor
  Compressor compressor = new Compressor(0);
  //defining wrist upper limit
  DigitalInput wristUpperLimit = new DigitalInput(1);


  //shooter one = canId 7
  TalonSRX shooterMotorOne = new TalonSRX(7); 
  //shooter two = canId 8
  TalonSRX shooterMotorTwo = new TalonSRX(8);
  
  //trigger motor = canId9
  TalonSRX triggerMotor = new TalonSRX(9);
  //index motor = canId10
  TalonSRX indexMotor = new TalonSRX(10);
  //wheelie intake motor = canID11
  TalonSRX wheelieInMotor = new TalonSRX(11);
  //intake manip motor = canID12
  TalonSRX intakeManipMotor = new TalonSRX(12);


  //1-3 = left side motors
  private final CANSparkMax leftFrontDrive = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax leftRearDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftTopDrive = new CANSparkMax(3, MotorType.kBrushless);
  
  //4-6 = right side motors
  private final CANSparkMax rightFrontDrive = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax rightRearDrive = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax rightTopDrive = new CANSparkMax(6, MotorType.kBrushless);

  // Groups the left and right side motors
  private final SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftRearDrive, leftFrontDrive, leftTopDrive);
  private final SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightRearDrive, rightFrontDrive, rightTopDrive);

  //tells the differential drive function what groups are what
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftDrive, rightDrive);

  //changing button board buttons from numbers to names
  public JoystickButton bBoardA = new JoystickButton(buttonBoard, 1);
  public JoystickButton bBoardB = new JoystickButton(buttonBoard, 2);
  public JoystickButton bBoardX = new JoystickButton(buttonBoard, 3);
  public JoystickButton bBoardY = new JoystickButton(buttonBoard, 4);
  public JoystickButton bBoardLB = new JoystickButton(buttonBoard, 5);
  public JoystickButton bBoardRB = new JoystickButton(buttonBoard, 6);

  //changing steering wheel buttons from numbers to names
  public JoystickButton sStickForward = new JoystickButton(SteeringWheel, 6);
  public JoystickButton sStickReverse = new JoystickButton(SteeringWheel, 5);

  //speed for shooter wheel
  public double shooterSpeed = 1;
  //speed for trigger belt
  public double triggerSpeed = 1;
  //intake speed
  public double intakeSpeed = -.4;  
  //index speed
  public double indexSpeed = -1;
  //intake up position
  public int intakeUpPos = 0;
  //intake down position
  public int intakeDownPos = 2700;
  //speed for zeroing the intake wrist during testing
  public double intakeZeroSpeed = -.3;

  //spins the shooter wheel

  /*this is the math to get distace from the limelight
   *




  */
  public double a1 = Math.toRadians(37);
  public double a2 = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
  public double FEET = (0.75-8)/Math.tan(a1+a2);
  void shooterSpin()
  {
    // double RPM = FEET * Constants.RPMmult;
    shooterMotorOne.set(ControlMode.PercentOutput, shooterSpeed); 
    shooterMotorTwo.set(ControlMode.PercentOutput, shooterSpeed);
//shooterMotorTwo.follow(shooterMotorOne);
//explore follow

  }

  //disables shooter wheel
  void shooterStop()
  {
    shooterMotorOne.set(ControlMode.Disabled, 1); 
    shooterMotorTwo.set(ControlMode.Disabled, 1);
  }

  //feeds shooter wheel
  void triggerSpin()
  {
    triggerMotor.set(ControlMode.PercentOutput, triggerSpeed);
  }
  
  //spins shooter trigger belt away from shooter 
  void triggerSpinOut()
  {
    triggerMotor.set(ControlMode.PercentOutput, -triggerSpeed);
  }
  //stops trigger motor
  void triggerStop()
  {
    triggerMotor.set(ControlMode.Disabled, 1);
  }
  //intake in
  void intakeIn()
  {
    wheelieInMotor.set(ControlMode.PercentOutput, intakeSpeed);
  }

  void intakeOut()
  {
  wheelieInMotor.set(ControlMode.PercentOutput, -intakeSpeed);
  }
  //intake stop
  void intakeStop()
  {
    wheelieInMotor.set(ControlMode.Disabled, 1);
  }
  //spins index motors in
  void indexIn()
  {
    indexMotor.set(ControlMode.PercentOutput, indexSpeed);
  }
  //spins index motors out
  void indexOut()
  {
    indexMotor.set(ControlMode.PercentOutput, -intakeSpeed);
  }

  //stops index motors
  void indexStop()
  {
    indexMotor.set(ControlMode.Disabled, 1);
  }
  //spins indexer and wheelie intake
  void intakeballs()
  {
    indexIn();
    intakeIn();
  }

  //stops indexer and wheelie intake motors
  void intakeballsStop()
  {
    indexStop();
    intakeStop();
  }
  //spins indexer out
  void unJam()
  {
    indexOut();
    intakeOut();
  }
  //stops unJam function
  
  //puts intake up
  void intakeUp()
  {
    intakeManipMotor.set(ControlMode.Position, intakeUpPos);
  }
  //puts intake down
  void intakeDown()
  {
    intakeManipMotor.set(ControlMode.Position, intakeDownPos);
  }

  //this puts all motors in disabled mode
  void motorsStop()
  {
    shooterMotorOne.set(ControlMode.Disabled, 1); 
    shooterMotorTwo.set(ControlMode.Disabled, 1);
    triggerMotor.set(ControlMode.Disabled, 1);
    indexMotor.set(ControlMode.Disabled, 1);
  }

  //puts gearboxes in low gear
  void lowGear()
  {
    Shifter.set(Value.kForward);
  }

  //puts gearboxes in high gear
  void highGear()
  {
    Shifter.set(Value.kReverse);
  }

  //lines the robot up with the limelight and gets it at the right distance
  void limelightAim()
  {
    double desiredD = Feet() - 457.2;
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    // double heading_error = -tx;
    double heading_error;
    double distance_error;

    //prints limelight x to smart dashboard
    SmartDashboard.putNumber("LimelightX", tx);
    //prints limelight y to the smart dashboard
    //SmartDashboard.putNumber("LimelightX", ty);
     //tx = limelight 
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //ty = limelight y
    //heading error = tx
    heading_error = -tx;
      
    //this is arcade drive uses to get the robot the right distance from the high port
    //double speed_adjust = SpeedkP * distance_error - speedMin_command;
    //this is what arcade drive uses to line the robot up with the high port
    double steering_adjust = SteerkP * heading_error - steerMin_command;
    // This adjust the speed and how close it is with the sonrsensor
    // double speed_adjust = desiredD * heading_error - steerMin_command;
    //lines the robot up with the high port using speed adjust and steering adjust
    robotDrive.arcadeDrive(0, steering_adjust);

    
  }

  //lines the robot up and fires
  void aimAndFire()
  {
    Long timer = System.currentTimeMillis() - 500;
    while(System.currentTimeMillis() < timer)
    {
    limelightAim(); //lines up
    }

      Timer.delay(1);

      shooterSpin();// spins up the shooter wheel to desired speed
      Timer.delay(.8);
      indexIn();
      triggerSpin();
      Timer.delay(2); //this is time it taked to shoot all balls from indexer
      
      //stops index trigger and shooter
      indexStop();
      triggerStop();
      shooterStop();
    }

  
  //this function moves the wrist back to its upper limit and resets the encoder
  void wristZero()
  {
    while(wristUpperLimit.get())
    {
      intakeManipMotor.set(ControlMode.PercentOutput, intakeZeroSpeed);
    }

    //stops intakeManipMotor
    intakeManipMotor.set(ControlMode.Disabled, 0);
    //resets wrist encoder
    intakeManipMotor.setSelectedSensorPosition(0);




  }


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() 
  {

   
    m_robotContainer = new RobotContainer();

    //setting up usb camera
    UsbCamera Camera = CameraServer.getInstance().startAutomaticCapture();
    
    //setting camera resolution
    Camera.setResolution(980 / 4, 1020 / 4);
    

   
    //starts compressor
    compressor.start(); 

    leftRearDrive.setInverted(false); //inverts motor1
    leftFrontDrive.setInverted(true); //inverts motor2
    rightRearDrive.setInverted(true); //inverts motor5
    rightFrontDrive.setInverted(false); //inverts motor4

    rightDrive.setInverted(true); //inverts right motor group
    leftDrive.setInverted(true); //inverts left motors
    //inverts shootertwo
    shooterMotorTwo.setInverted(false);
    //inverts wrist motor
    intakeManipMotor.setInverted(true);
    
    intakeManipMotor.configFactoryDefault();
    //telling the intake manip where to look for pid constants
    intakeManipMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,Constants.kPIDloopIdx,Constants.kTimeoutMs);
    
    intakeManipMotor.setSensorPhase(Constants.ksensorPhase);
    //setting nominal forward output for wrist
    intakeManipMotor.configNominalOutputForward(0,Constants.kTimeoutMs);
    //setting nominal reverse output for wrist
    intakeManipMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    //setting peak forward output for wrist
    intakeManipMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    //setting perk reverse output for wrist
    intakeManipMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    //setting allowable error for intake manip
    intakeManipMotor.configAllowableClosedloopError(0,Constants.kPIDloopIdx,Constants.kTimeoutMs);
    //telling intake manip where to get pid values 
    intakeManipMotor.config_kF(Constants.kPIDloopIdx,Constants.kGains.kF,Constants.kTimeoutMs);
    intakeManipMotor.config_kP(Constants.kPIDloopIdx,Constants.kGains.kP,Constants.kTimeoutMs);
    intakeManipMotor.config_kI(Constants.kPIDloopIdx,Constants.kGains.kI,Constants.kTimeoutMs);
    intakeManipMotor.config_kD(Constants.kPIDloopIdx,Constants.kGains.kD,Constants.kTimeoutMs);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();

    if(wristUpperLimit.get())
    {
      
    }
    else
    {
      intakeManipMotor.setSelectedSensorPosition(0);
    }

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() 
  {

  }

  @Override
  public void disabledPeriodic() 
  {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //zeros wrist encoder when teleop is enabled
    wristZero();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 

  {

  SmartDashboard.putBoolean("highGear", false);
  SmartDashboard.putBoolean("lowGear", false);


    //combines the pedals 2 axis into 1 positive and negative axis
    double driveSpeed = SteeringWheel.getRawAxis(3) - SteeringWheel.getRawAxis(2);

    //arcade drive using pedals for speed control and steering wheel for steering
    robotDrive.arcadeDrive(-driveSpeed, - SteeringWheel.getX(Hand.kLeft) *.6);
    SmartDashboard.putNumber("drivespeed", driveSpeed);

    //prints encoder ticks to smartdashboard
    SmartDashboard.putNumber("wristTicks", intakeManipMotor.getSelectedSensorPosition());

    SmartDashboard.putBoolean("limitswitch", wristUpperLimit.get());

    SmartDashboard.putNumber("shootersonar", shooterSonar.getVoltage());




    
    
    
    // if buttonBoard b is pressed, run aim and fire
    if(bBoardB.get())
    {
      indexIn();
      triggerSpin();
    }
    else if(bBoardRB.get())
    {
      indexOut();
      triggerSpinOut();
    }
    else if(bBoardY.get())
    {
      shooterSpin();
    }
    else
    {
      shooterStop();
      triggerStop();
      indexStop();
    }
    
    //puts intake up if button board joystick is up
    if(bBoardX.get())
    {
      SmartDashboard.putBoolean("intakeup", true); //this is for troubleshooting
      intakeUp();
    }
    else 
    {
      SmartDashboard.putBoolean("intakeup", false); //this is for troubleshooting
    }
    //puts intake down if button board joystick is down
    if(bBoardA.get())
    {
      SmartDashboard.putBoolean("intakedown", true);//this is for troubleshooting
      SmartDashboard.putNumber("wristTicks", intakeManipMotor.getSelectedSensorPosition());

      intakeDown();
    }
    else
    {
      
      SmartDashboard.putBoolean("intakedown", false);//this is for troubleshooting
    }
    //if steering stick is foreward, shift to high gear
    if(sStickForward.get())
    {
      highGear();
      SmartDashboard.putBoolean("lowGear", true);
    } 
    //if steering stick is reverse, shift to low gear
    if(sStickReverse.get())
    {
      lowGear();
      SmartDashboard.putBoolean("lowGear", true);
    }

    if(bBoardLB.get())
    {
      aimAndFire();
    }
    

    
   
    //spins wheelie intake if wrist is down
    if (intakeManipMotor.getSelectedSensorPosition() > 2000)
    {
      intakeIn();
    }
    else
    {
      intakeStop();
    }

  }


  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() 
  {

  }
}
