// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.controllerConstants;
import frc.robot.Constants.motorConstants;
import frc.robot.Constants.speedConstants;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;  

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Compressor;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

// YEEEEEEET

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
  public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public String autoToDo;

  //private RobotContainer m_robotContainer;

// this is where the initialization starts 🐌🐌🐌🐌🐌🐌🐌🐌🐌

// the arm arg
   private final CANSparkMax ExtendoPatronum = new CANSparkMax(5, MotorType.kBrushed); 
   private final CANSparkMax WEEEWOOOO = new CANSparkMax(6, MotorType.kBrushed);

// ANALOOOOOOOONG STUUUUUUUUUff🧨🎇🎇🧨🎇🧨✨✨
   private final DigitalOutput LED = new DigitalOutput(0);

// limit switches
   private final DigitalInput extendoLimit = new DigitalInput(7);
   private final DigitalInput lazerbeam = new DigitalInput(2);
  

// pnumatics
   private final Compressor pcmCompressor = new Compressor(0,PneumaticsModuleType.CTREPCM);
   private final DoubleSolenoid PKSHHH = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);
   private final DoubleSolenoid GetUsOnChargeStation = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 6, 7);

//gyro
   private final AHRS gyro = new AHRS(I2C.Port.kOnboard);

//timer
   private final Timer timer = new Timer();


   private static final String DefaultAuto = "Default";
   private static final String ChargeAuto = "Charge Station";
   private static final String noAuto = "NO Autonomous";
   private static final String testStendo = "Rotation Test";
   private String m_autoSelected;
   private final SendableChooser<String> m_chooser = new SendableChooser<>();


  /*  private static final double cpr = 1024; // this is different for different types of this encoder
  private static final double WHEELbig = 8;  // 8 bc 8 inch diameter
  private final Encoder m_frontLeftEncoder = new Encoder(0, 1);
  private final Encoder m_frontRightEncoder = new Encoder(2, 3);
  private final Encoder m_backLeftEncoder = new Encoder(4, 5);
  private final Encoder m_backRightEncoder = new Encoder(6, 7); */

// feed forward?????????????????
  // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);



/*** This function is run when the robot is first started up and should be used for any
* initialization code.*/
  @Override
  public void robotInit() {
// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
// autonomous chooser on the dashboard.
    //m_robotContainer = new RobotContainer();


    m_chooser.setDefaultOption("Default",DefaultAuto);
    m_chooser.addOption("Charge Station",ChargeAuto);
    m_chooser.addOption("NO Autonomous", noAuto);
    m_chooser.addOption("Rotation Test", testStendo);
    SmartDashboard.putData("Auto Choice", m_chooser);
    gyro.calibrate();

    /*m_frontLeftEncoder.setDistancePerPulse(Math.PI*WHEELbig/cpr);
    m_frontRightEncoder.setDistancePerPulse(Math.PI*WHEELbig/cpr);
    m_backLeftEncoder.setDistancePerPulse(Math.PI*WHEELbig/cpr);
    m_backRightEncoder.setDistancePerPulse(Math.PI*WHEELbig/cpr); */


  }



/**
* This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
* that you want ran during disabled, autonomous, teleoperated and test.
*
* <p>This runs after the mode specific periodic functions, but before LiveWindow and
* SmartDashboard integrated updating.
*/
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Arm Limit", extendoLimit.get());
    SmartDashboard.putBoolean("phota eye", lazerbeam.get());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("heading", gyro.getCompassHeading());
    /*   double distance = m_frontLeftEncoder.getDistance();
    SmartDashboard.putNumber("Encoder LF", distance);
    double distance2 = m_frontRightEncoder.getDistance();
    SmartDashboard.putNumber("Encoder LB", distance2);
    double distance3 = m_backLeftEncoder.getDistance();
    SmartDashboard.putNumber("Encoder RF", distance3);
    double distance4 = m_backRightEncoder.getDistance();
    SmartDashboard.putNumber("Encoder RB", distance4);  */

// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
// commands, running already-scheduled commands, removing finished or interrupted commands,
// and running subsystem periodic() methods.  This must be called from the robot's periodic
// block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }



/** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}



//idk
  @Override
  public void disabledPeriodic() {}



/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //gyro.calibrate();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_autoSelected = m_chooser.getSelected();
    
    System.out.println("auto selected: " + m_autoSelected);

    timer.reset();
    timer.start();

    WEEEWOOOO.setOpenLoopRampRate(0.25);
    //autoToDo = m_robotContainer.getAutoChoice();

  }



/** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

//gyro reading
  SmartDashboard.putNumber("Auto Gyro Pitch", gyro.getPitch());

//charge station

if(m_autoSelected == "Default"){

      closClaw(0.0, 2.3);

      armRotU(speedConstants.aSpep, 0.0, 1.6);

      stendo(speedConstants.armSpep, 0.6, 1.6);

      stopArmRot(1.6, 6.7);

      stopStendo(1.6, 3.3);

      movingFBG(speedConstants.aMovingSpep, 1.7, 2.25, 0.5, -0.5);

      stopping(2.25, 3.0);

      eopenClaw(2.3, 9.25); 

      evilStendo(speedConstants.armSpep, 2.5, 6.0);

      movingFBG(-0.7, 2.5, 4.05, 0.5, -0.5);

      armRotD(speedConstants.aSpep, 3, 4.0);

      stopArmRot(4.0, 8);

      stopStendo(6, 15.0);

      movingLG(speedConstants.aMovingSpep, 4.05, 7.2, -100, -175, -183, -180.5, -179.5);

      strafingLR(speedConstants.aMovingSpep, 7.2, 7.7);

      stendo(speedConstants.aMovingSpep, 7.0, 7.6);

      movingFBG(.25, 7.8, 8, -179.5,-180.5);

      armRotD(0.4, 8, 10.0);

      closClaw(9.25, 13.95);

      armRotU(0.5, 9.45, 12.0);

      stopArmRot(12.0,15);

      movingRG(-speedConstants.aMovingSpep, 9.55, 12.4, -80, -5, 5, -1, 1);

      movingFBG(1, 12.45, 13.7, 0.5, -0.5);

      movingFBG(.4, 13.7, 14.8, 0.5, -0.5);

      eopenClaw(14.8, 15.0);



} 
 
if(m_autoSelected == "Charge Station"){
  closClaw(0.0, 3.1);

  armRotU(speedConstants.aSpep, 0.0, 1.6);

  stendo(speedConstants.armSpep, 0.5, 2.5);

  stopArmRot(1.6, 6.7);

  stopStendo(2.5, 3.3);

  movingFB(speedConstants.aMovingSpep, 2.5, 3.0);

  stopping(3.0, 5.2);

  eopenClaw(3.3, 15.0); 

  evilStendo(speedConstants.armSpep, 3.4, 5.3);

  movingFB(-0.85, 5.2, 6.5);

  armRotD(speedConstants.aSpep, 5.4, 6.7);

  stopArmRot(6.7, 15.0);

  stopStendo(5.3, 15.0);

  gyroFunc(6.5, 15.0);


  }  

  if(m_autoSelected == "Rotation Test"){

    evilStendo(speedConstants.aMovingSpep, 0.0, 2.0);

    stendo(speedConstants.aMovingSpep, 3.0, 3.6);

    stopStendo(3.6, 15.0);

    armRotD(0.1, 4.0, 6.0);

      
  }

    if(m_autoSelected == "NO Autonomous"){

      
    }
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

//speedramps
    motorConstants.leftMotorFront.setOpenLoopRampRate(0.25);
    motorConstants.leftMotorBack.setOpenLoopRampRate(0.25);
    motorConstants.rightMotorFront.setOpenLoopRampRate(0.25);
    motorConstants.rightMotorBack.setOpenLoopRampRate(0.25);
    WEEEWOOOO.setOpenLoopRampRate(0.25);
    ExtendoPatronum.setOpenLoopRampRate(0.1);
  }

  private boolean startButton = true;
  private double upDownMoved = 0;

  public MecanumDrive theDrivingBoy = new MecanumDrive(motorConstants.leftMotorFront, motorConstants.leftMotorBack
    , motorConstants.rightMotorFront, motorConstants.rightMotorBack);

   

/** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    

  SmartDashboard.putNumber("CLAW controller turning", controllerConstants.controllerTHECLAWWWWW.getPOV()); 
  //SmartDashboard.putNumber("gyro pitch", gyro.getPitch()); 

// code for go   🚗🚗🚗🚗🚗🚗🚗🚗
  if(startButton){

    if(controllerConstants.controllerVROOMVROOM.getLeftBumper()) {
    theDrivingBoy.driveCartesian((-controllerConstants.controllerVROOMVROOM.getRawAxis(1)), 
    controllerConstants.controllerVROOMVROOM.getRawAxis(0),
    controllerConstants.controllerVROOMVROOM.getRawAxis(4));

    } else if(controllerConstants.controllerVROOMVROOM.getRightBumper()){
    theDrivingBoy.driveCartesian((-controllerConstants.controllerVROOMVROOM.getRawAxis(1)/-speedConstants.kMaxSpeed), 
    controllerConstants.controllerVROOMVROOM.getRawAxis(0)/-speedConstants.kMaxSpeed,
    controllerConstants.controllerVROOMVROOM.getRawAxis(4)/-speedConstants.kMaxSpeed);

    } else {
    theDrivingBoy.driveCartesian((-controllerConstants.controllerVROOMVROOM.getRawAxis(1)/speedConstants.kMaxSpeed), 
    controllerConstants.controllerVROOMVROOM.getRawAxis(0)/speedConstants.kMaxSpeed,
    controllerConstants.controllerVROOMVROOM.getRawAxis(4)/speedConstants.kMaxSpeed);
    }

//invert arm rotation
  WEEEWOOOO.setInverted(true);

//set axis for arm stuff
  upDownMoved = controllerConstants.controllerTHECLAWWWWW.getRawAxis(5);

//led
  LED.set(true);

// arm controlling stuuuuffff 🙋‍♀️🙋‍♂️🙋‍♀️🙋‍♂️🙋‍♀️🙋‍♂️🙋‍♀️🙋‍♂️
  WEEEWOOOO.set((upDownMoved)/speedConstants.armSpep);

//extendo limit switch
  if (extendoLimit.get() == false && controllerConstants.controllerTHECLAWWWWW.getRawAxis(1) > 0) {
    ExtendoPatronum.set(0);
  } 
  else if(extendoLimit.get() == true || extendoLimit.get() == false && controllerConstants.controllerTHECLAWWWWW.getRawAxis(1) < 0){
    ExtendoPatronum.set(MathUtil.applyDeadband(controllerConstants.controllerTHECLAWWWWW.getRawAxis(1)
    , 0.4));
  }

// pnematic code I choose you
  if(controllerConstants.controllerTHECLAWWWWW.getLeftBumper()) {
    PKSHHH.set(DoubleSolenoid.Value.kForward);
  }   
  else if(controllerConstants.controllerTHECLAWWWWW.getRightBumper())  {
    PKSHHH.set(DoubleSolenoid.Value.kReverse);
  }

  }
  }



  @Override
  public void testInit() {
// Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }



/** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}



/** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}



/** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}



//forward backward
  public void movingFB(double speed, double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      motorConstants.leftMotorFront.set(speed);
      motorConstants.leftMotorBack.set(speed);
      motorConstants.rightMotorFront.set(speed);
      motorConstants.rightMotorBack.set(speed);
    }
  }

  public void movingFBG(double speed, double lowT, double highT, double hiDegrees, double loDegrees){
    if(timer.get() > lowT && timer.get() < highT){

          if(gyro.getAngle() > hiDegrees){
            motorConstants.leftMotorFront.set(speed - 0.3);
            motorConstants.leftMotorBack.set(speed - 0.3);
            motorConstants.rightMotorFront.set(speed);
            motorConstants.rightMotorBack.set(speed);

    } else
    if(gyro.getAngle() < loDegrees){
      motorConstants.leftMotorFront.set(speed);
      motorConstants.leftMotorBack.set(speed);
      motorConstants.rightMotorFront.set(speed - 0.2);
      motorConstants.rightMotorBack.set(speed - 0.2);

    } else{
      motorConstants.leftMotorFront.set(speed);
      motorConstants.leftMotorBack.set(speed);
      motorConstants.rightMotorFront.set(speed);
      motorConstants.rightMotorBack.set(speed);
    }
  }
  }

  

//stop
  public void stopping(double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      motorConstants.leftMotorFront.stopMotor();
      motorConstants.leftMotorBack.stopMotor();
      motorConstants.rightMotorFront.stopMotor();
      motorConstants.rightMotorBack.stopMotor();
    }
  }

//turning (right is positive and left is negative)
  public void movingLR(double speed, double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      motorConstants.leftMotorFront.set(-speed);
      motorConstants.leftMotorBack.set(-speed);
      motorConstants.rightMotorFront.set(speed);
      motorConstants.rightMotorBack.set(speed);
    }
  }

  public void movingLG(double speed, double lowT, double highT
  , double thresh1, double slowDown, double bounceBack, double lowRange, double highRange){
    if(timer.get() > lowT && timer.get() < highT){

      // move to this
        if(gyro.getAngle() > thresh1){
          movingLR(speed, lowT, highT);

          //slow down
        } else if(gyro.getAngle() < thresh1 && gyro.getAngle() > slowDown){
          movingLR(0.3, lowT, highT);}

          // bounce back
     else if(gyro.getAngle() < bounceBack){
       movingLR(-0.25, lowT, highT);  
      } 
       
         }else if(gyro.getAngle() > lowRange && gyro.getAngle() < highRange){ 
            stopping(lowT, highT);
         }
        }


        public void movingRG(double speed, double lowT, double highT
  , double thresh1, double slowDown, double bounceBack, double lowRange, double highRange){
    if(timer.get() > lowT && timer.get() < highT){

      // move to this
        if(gyro.getAngle() < thresh1){
          movingLR(speed, lowT, highT);

          //slow down
        } else if(gyro.getAngle() > thresh1 && gyro.getAngle() < slowDown){
          movingLR(-0.3, lowT, highT);}

          // bounce back
     else if(gyro.getAngle() > bounceBack){
       movingLR(0.25, lowT, highT);  
      } 
       
         }else if(gyro.getAngle() > lowRange && gyro.getAngle() < highRange){ 
            stopping(lowT, highT);
         }
        }
  
  


//strafing (right is positive and left is negative)
  public void strafingLR(double speed, double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      motorConstants.leftMotorFront.set(speed);
      motorConstants.leftMotorBack.set(-speed);
      motorConstants.rightMotorFront.set(-speed);
      motorConstants.rightMotorBack.set(speed);
    }
  }

// beginning of stendo funcs
  public void stendo(double speed, double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      ExtendoPatronum.set(-speed);
    }
  }

  public void evilStendo(double speed, double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      if (extendoLimit.get() == false) {
        ExtendoPatronum.set(0);}
      else{ExtendoPatronum.set(speed);}
    }
  }
  
  public void stopStendo(double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      ExtendoPatronum.stopMotor();
    }
  }
// end of stendo funcs

// start of arm rotation funcs
  public void armRotU(double speed, double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      WEEEWOOOO.set(-speed);
    }
  }

  public void armRotD(double speed, double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){

      if (lazerbeam.get() == false) {
          WEEEWOOOO.set(0);}
      else{WEEEWOOOO.set(speed);}
    }
  }
  
  public void stopArmRot(double lowT, double highT){
    if(timer.get() > lowT && timer.get() < highT){
      WEEEWOOOO.stopMotor();
    }
  }
// end of arm rotation funcs

// start of claw funcs
  public void eopenClaw(double lowT, double highT) {
    if(timer.get() > lowT && timer.get() < highT){
      PKSHHH.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void closClaw(double lowT, double highT) {
    if(timer.get() > lowT && timer.get() < highT){
      PKSHHH.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void deploy(double lowT, double highT){

    if(timer.get() > lowT && timer.get() < highT){
      GetUsOnChargeStation.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void unDeploy(double lowT, double highT){

    if(timer.get() > lowT && timer.get() < highT){
      GetUsOnChargeStation.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void gyroFunc(double lowT, double highT){

    if(timer.get() > lowT && timer.get() < highT) {

       
          if(Math.abs(gyro.getPitch()) > 12){
        movingFB(Math.copySign(0.28, gyro.getPitch()), 0, 60);
          } 
        
           else if(Math.abs(gyro.getPitch()) > 7){
            movingFB(0.2, 0, 60);
          } 
          else if(Math.abs(gyro.getPitch()) > 3){
            movingFB(Math.copySign(0.15, gyro.getPitch()), 0, 60);
          } 
          else {
            stopping(0, 60);
          }
        }
    }

  }


  
// endo of claw funcs
