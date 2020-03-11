/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.lang.Math;



import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String findEncoderValue = "Encoder Value Finder";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Spark FRM,FLM,BLM,BRM,intake,winch,arm;
  Joystick Joy1,Joy2,Joy3;
  Encoder FRME,FLME,BRME,BLME, armEncoder;
  Compressor compressor;
  DoubleSolenoid leftarmpiston,rightarmpiston;
  Spark door;
  boolean isBusy = true;

  private final double TurnWheelDiameter = wheelradius*2; // Measure
  private final double InchPerRev = TurnWheelDiameter/1;
  private final double PulsesPerDegree = PulsesPerRev*(1/360);

  final static double PulsesPerRev = 1024; // Unknown
  final static double wheelradius = 3; // Calculated in inches
  final static double WheelCirc = Math.PI*(wheelradius*2);
  final static double InchPerPulse = (WheelCirc) / PulsesPerRev; // Calculated in inches, should be in meters
  CameraServer cam0;
  ADXRS450_Gyro gyro;
  SpeedControllerGroup leftSide, rightSide;
  DifferentialDrive robot;
  Spark Blinkin;
  Timer pistonCOOLDOWN, doorCOOLDOWN;
  final static double COOLDOWN = 0.5;
  String gameMessage;
  DriverStation DS;
  double armposition = 0;
  boolean maintainArm = false;
  final static double armspeed = 0.3;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Encoder Value Finder", findEncoderValue);
    SmartDashboard.putData("Auto choices", m_chooser);
    FRM = new Spark(2);
    FLM = new Spark(0);
    BRM = new Spark(3);
    BLM = new Spark(1);
    Blinkin = new Spark(4);
    intake = new Spark(5);
    winch = new Spark(6);
    arm = new Spark(7);
    FLME = new Encoder(0,1);
    FRME = new Encoder(2,3);
    BRME = new Encoder(4,5);
    BLME = new Encoder(6,7);
    armEncoder = new Encoder(8,9);
    Joy1 = new Joystick(0);
    Joy2 = new Joystick(1);
    Joy3 = new Joystick(2);
    gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

    compressor = new Compressor();
    leftarmpiston = new DoubleSolenoid(0,1);
    rightarmpiston = new DoubleSolenoid(2,3);
    pistonCOOLDOWN = new Timer();
    doorCOOLDOWN = new Timer();
    door = new Spark(8);
    CameraServer.getInstance().startAutomaticCapture();

    
    leftSide = new SpeedControllerGroup(FLM, BLM);
    rightSide = new SpeedControllerGroup(FRM,BRM);
    robot = new DifferentialDrive(leftSide, rightSide);
    //odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()));
    leftSide.setInverted(true);
    FLME.setReverseDirection(true);
    BLME.setReverseDirection(true);

    armEncoder.setDistancePerPulse(InchPerPulse);
    FLME.setDistancePerPulse(InchPerPulse);
    FRME.setDistancePerPulse(InchPerPulse);
    BLME.setDistancePerPulse(InchPerPulse);
    BRME.setDistancePerPulse(InchPerPulse);
    gyro.calibrate();
    gameMessage = DriverStation.getInstance().getGameSpecificMessage();
    DS = DriverStation.getInstance();
    leftarmpiston.set(Value.kReverse);
    rightarmpiston.set(Value.kReverse);
    
    


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //Displays the alliance colors.
    if(DS.getAlliance() == Alliance.Blue && Blinkin.getSpeed() != 0.87){
      //Turns it blue
      Blinkin.setSpeed(0.87);
    } else if(DS.getAlliance() == Alliance.Red && Blinkin.getSpeed() != 0.61){
      // Turns it red
      Blinkin.setSpeed(0.61);
    }
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    turnRight(90);
    stopBot();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double left = Joy1.getRawAxis(1);
    double right = Joy2.getRawAxis(1);
    double Joy3Y = Joy3.getRawAxis(1);

    //Maintains the arm positition if told to
    if(maintainArm && Joy3Y <= 0.7){
      if(armEncoder.getDistance() > armposition){
        arm.setSpeed(-armspeed);
      } else {
        arm.setSpeed(armspeed);
      }
    }
    //If the driver holds the first button, the robot will drive at full power
    if(Joy1.getRawButtonPressed(1) || Joy2.getRawButtonPressed(1)){
      robot.tankDrive(left, right);
    } else{
      robot.tankDrive(left/2, right/2);
    }
    //If the first button is pressed, pistons extend the arm to reach the rendezous bar or retract
    if((Joy1.getRawButtonPressed(2) || Joy2.getRawButtonPressed(2)) && pistonCOOLDOWN.get() > COOLDOWN){
      pistonCOOLDOWN.reset();
      if(leftarmpiston.get() == Value.kForward){
        leftarmpiston.set(Value.kReverse);
        rightarmpiston.set(Value.kReverse);
      } else{
        leftarmpiston.set(Value.kForward);
        rightarmpiston.set(Value.kForward);
      }
    }
    //If the second button is pressed, the servo opens and closes the door of the intake
    if(Joy3.getRawButtonPressed(2) && doorCOOLDOWN.get() > COOLDOWN){
      doorCOOLDOWN.reset();
      door.setSpeed(-0.5);

    } else if(Joy3.getRawButtonPressed(3) && doorCOOLDOWN.get() > COOLDOWN){
      doorCOOLDOWN.reset();
      door.setSpeed(0.5);
    } else{
      door.setSpeed(0);
    }

    //If the third button is pressed then the intake turns on
    if(Joy3.getRawButtonPressed(1)){
      intake.setSpeed(0.5);
    } else{
      intake.setSpeed(0);
    }
    //If the fifth button is pressed then the wench spins forward, if the fourth is pressed it spins backward
    if(Joy3.getRawButtonPressed(5)){
      winch.setSpeed(0.5);
    } else if(Joy3.getRawButtonPressed(4)){
      winch.setSpeed(-0.5);
    }else{
      winch.setSpeed(0);
    }
    //If the seventh button is pressed then the arm turns forward, the eighth button makes it go in reverse
    if(Joy3Y > 0.7){
      arm.setSpeed(0.5);
      armposition = armEncoder.getDistance();
      maintainArm = true;
    } else if(Joy3Y < 0.3){
      arm.setSpeed(-0.5);
      maintainArm = false;
    } else if(!maintainArm){
      arm.setSpeed(0);
    }
    SmartDashboard.putNumber("Degrees", gyro.getAngle());


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }
  /**
   * Uses given left and right side speeds and drives to a specifc target distance.
   * @param leftSpeed Speed of the left motors
   * @param rightSpeed Speed of the right motors
   * @param target Target distance in inches
   */
  public void EncoderDrive(double leftSpeed, double rightSpeed, double target){
    resetEncoders();
    robot.tankDrive(leftSpeed, rightSpeed);
    while(isBusy){
      SmartDashboard.putNumber("Left Distance To Travel", target - FLME.getDistance());
      SmartDashboard.putNumber("Right Distance To Travel", target - FRME.getDistance());

      checkEncoder(FLME, leftSpeed, target);
      checkEncoder(FRME, rightSpeed, target);
      checkEncoder(BLME, leftSpeed, target);
      checkEncoder(BRME, rightSpeed, target);
    }
    resetEncoders();

  }

  
  /**
   * 
   */
  public void trenchAuto(){
    EncoderDrive(0.75, 0.75, 389);//Drives under the trench
    turnLeft(90); // Check
    EncoderDrive(0.75, 0.75, 67);//Drives to spot on the initiation line
    turnRight(90);
    EncoderDrive(0.75, 0.75, 110);//Lines up for the power port
    EncoderDrive(0.25, 0.25, 29);//30" drive up
  }

  /**
   * 
   */
  public void rendezvousAuto(){
    EncoderDrive(0.75, 0.75, 130.5);//Drives to rendezvous point
    turnLeft(45); // Check
    EncoderDrive(0.75, 0.75, 167);//Drives under rendezvous spot
    turnRight(45);
    EncoderDrive(0.75, 0.75, 88);//Drives to initiation line
    turnRight(90); // Check
    EncoderDrive(0.75, 0.75, 67);//Drives to spot on the initiation line
    turnLeft(90);
    EncoderDrive(0.75, 0.75, 110);//Lines up for the power port
    EncoderDrive(0.25, 0.25, 29);//30" drive up
  }
  /**
   * This allows the robot to turn right by finding target Degrees and gives what angle it is currently at.
   * @param targetDegrees gets out degrees the robot wants to go right 
   * 
   */
  public void turnRight(double targetDegrees){
    double currentDegrees = gyro.getAngle();
    while((currentDegrees > targetDegrees+2) || (currentDegrees < targetDegrees-2)){
      currentDegrees = gyro.getAngle();
      robot.tankDrive(0.25, -0.25);
    }
  }
/**
 * This allows the robot to turn left by finding target Degrees and gets what angle it is currently at.
 * @param targetDegrees gets the degrees the robot wants to go left.
 */
  public void turnLeft(double targetDegrees){
    double currentDegrees = gyro.getAngle();
    while((currentDegrees > targetDegrees+2) || (currentDegrees < targetDegrees-2)){
      currentDegrees = gyro.getAngle();
      robot.tankDrive(-0.25, 0.25);
    }
  }
  /**  
   * This will stop the bot when activated
   **/
  public void stopBot(){
    robot.tankDrive(0, 0);
  }
  private void resetEncoders(){
    FLME.reset();
    FRME.reset();
    BLME.reset();
    BRME.reset();
  }
  private void checkEncoder(Encoder encoder, double speed, double target){
    if(speed < 0){ // If right side is going back
      if(encoder.getDistance() <= -target){ // If the encoder has passed the target
        isBusy = false;
      }
    } else{ // If right side is going forward
      if(encoder.getDistance() >= target){ // If the encoder has passed the target
        isBusy = false;
      }
    }
  }
}
