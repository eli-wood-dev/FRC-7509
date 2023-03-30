/*
  2022 everybot code
  written by carson graf 
  don't email me, @ me on discord
*/

/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // DONE: check if change is needed (comment this out?)

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController; // added PG 29 May 2022
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.*;

public class Robot extends TimedRobot {

  UsbCamera frontCam;
  UsbCamera backCam;
  NetworkTableEntry cameraSelection;

  int currentCam = 0;
  
  //Definitions for the hardware. Change this if you change what stuff you have plugged in
  //Mostly adjusted to correct Can IDs
  CANSparkMax driveLeftA  = new CANSparkMax(Constants.CAN_LEFT_A,  MotorType.kBrushless);
  CANSparkMax driveLeftB  = new CANSparkMax(Constants.CAN_LEFT_B,  MotorType.kBrushless);
  CANSparkMax driveRightA = new CANSparkMax(Constants.CAN_RIGHT_A, MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(Constants.CAN_RIGHT_B, MotorType.kBrushless);
  CANSparkMax arm         = new CANSparkMax(Constants.CAN_ARM,     MotorType.kBrushless);
  
  //WPI_TalonFX intake    = new WPI_TalonFX(1, "rio"); // DONE: check if change is needed
  CANSparkMax intake      = new CANSparkMax(Constants.CAN_SHOOTER, MotorType.kBrushless);

  Joystick driverController         = new Joystick(      Constants.USB_DRIVER_CONTROLLER);
  XboxController operatorController = new XboxController(Constants.USB_OPERATOR_CONTROLLER);

  DigitalInput upLimitSwitch   = new DigitalInput(9);
  DigitalInput downLimitSwitch = new DigitalInput(0); // subject to change

  RelativeEncoder encoderLeftA  = driveLeftA.getEncoder();
  RelativeEncoder encoderLeftB  = driveLeftB.getEncoder();
  RelativeEncoder encoderRightA = driveRightA.getEncoder();
  RelativeEncoder encoderRightB = driveRightB.getEncoder();
  RelativeEncoder encoderArm    = arm.getEncoder();

  // DONE: check if change is needed -- add code here for shooter/intake?
  RelativeEncoder encoderIntake = intake.getEncoder();

  PowerDistribution pdp         = new PowerDistribution();

  // Constants for controlling the arm. consider tuning these for your particular robot
  final double armHoldUp = 0.03;
  final double armHoldDown = 0.13;
  final double armTravel = 0.3;

  final double armTimeUp = 0.5;
  final double armTimeDown = 0.35;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0;

  double autoStart = 0;
  boolean goForAuto = false;

  // Controls for the arm
  double initialArmPosition;
  boolean isUp;
  boolean isDown;
  boolean isMovingUp;
  boolean isMovingDown;
  boolean isArmStopped;

  //Speed ramping
  private static double rampBand = Constants.DEFAULT_RAMP_BAND; //EW 21 sep 2022
  private static double previousMotorSpeed = 0.0; //EW 21 sep 2022


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // set motor defaults for NNEO Brushless Motors

    driveLeftA. restoreFactoryDefaults();
    driveLeftB. restoreFactoryDefaults();
    driveRightA.restoreFactoryDefaults();
    driveRightB.restoreFactoryDefaults();
    arm.        restoreFactoryDefaults();

    // DONE: check if change is needed -- add code here for shooter/intake?
    intake.     restoreFactoryDefaults();

    //Camera
    
    frontCam = CameraServer.startAutomaticCapture(0); //EW 26 sep 2022
    backCam = CameraServer.startAutomaticCapture(1);

    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
  
    //Configure motors to turn correct direction. You may have to invert some of your motors

    driveLeftA. clearFaults();
    driveLeftB. clearFaults();
    driveRightA.clearFaults();
    driveRightB.clearFaults(); 
    driveLeftA. setInverted(true);
    driveLeftB. setInverted(true);
    driveRightA.setInverted(false);
    driveRightB.setInverted(false);
    arm.        setInverted(false);

    // DONE: check if change is needed -- add code here for shooter/intake?
    intake.     clearFaults();
    intake.     setInverted(false);
    intake.     setClosedLoopRampRate(Constants.INTAKE_RAMP);

    arm.setIdleMode(IdleMode.kBrake);
    // TO DO: check if change is needed -- add code here for shooter/intake?

    driveLeftA. burnFlash();
    driveLeftB. burnFlash();
    driveRightA.burnFlash();
    driveRightB.burnFlash();
    arm.        burnFlash();

    // DONE: check if change is needed -- add code here for shooter/intake?
    intake.     burnFlash();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

    // check arm sensor position
    initialArmPosition = encoderArm.getPosition();
    isUp               = true; // always start a match with arm up
    isDown             = false;
    isMovingUp         = false;
    isMovingDown       = false;
    isArmStopped       = false;
    
    reportStatus();

  } // end robotInit()

  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    //goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    goForAuto = true; //Ew 20 Oct 2022
    SmartDashboard.putBoolean("Go For Auto", true); //Ew 20 Oct 2022
  }

  /** This function is called periodically during autonomous.
   * requires switching to limit switches from timer
  */
  @Override
  public void autonomousPeriodic() {
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false); //Ew 20 Oct 2022
    // System.out.println(goForAuto);

    //arm control code. same as in teleop
    /* if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldUp);
      }
    } */
    
    //get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if(goForAuto){
      //series of timed events making up the flow of auto
      //score stored ball?
      if(autoTimeElapsed < 3.0) {
        //spit out the ball for three seconds
        intake.set(Constants.SHOOTER_SPEED);  // wrong way
        System.out.println("time: " + autoTimeElapsed + " shooting");
      } // end if
      else if(autoTimeElapsed < 5.5)  { //reduced time by 1 EW 29 Oct
        //stop spitting out the ball and drive backwards *slowly* for two seconds
        //leave enclosure
        
        driveLeftA. set(0.2); // EW 29 Oct .3 to .2
        driveLeftB. set(0.2);
        driveRightA.set(0.2);
        driveRightB.set(0.2);

        intake.     set(0.0);  // DONE: check if change is needed
        System.out.println("time: " + autoTimeElapsed + " driving back");
      } // end else if
      else {
        //do nothing for the rest of auto
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);

        intake.set(0);  // DONE: check if change is needed
      } // end else
    } // end if

    // *** report device status

    reportStatus();

  } // end AutonomousInit()

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    reportStatus();

  } // end teleopInit()

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Switching the camera
    
    if (driverController.getTriggerReleased()) {
      if (currentCam == 0) {
        cameraSelection.setString(backCam.getName());
        currentCam = 1;
      } else {
        cameraSelection.setString(frontCam.getName());
        currentCam = 0;
      }
    }
    
    //Set up arcade steer
    double forward = driverController.getRawAxis(1); // y axis id negative forwards
    double turn    = driverController.getRawAxis(0);  // x axis

    // define throttle
    // EW 20 Sep 2020 
    
    double throttle = Math.min(-driverController.getRawAxis(3), Constants.MAX_THROTTLE);
    throttle = (throttle + 1) / 2;
    double turnThrottle = Math.min(throttle, Constants.TURN_THROTTLE);

    // apply rampband only to forward
    forward  = applyRampBand(forward); //Ew 11 Oct 2022

    // apply forward deadband
    if(-Constants.FORWARD_DEADBAND < forward && forward < Constants.FORWARD_DEADBAND) {
      forward = 0.0;
    } // end if
    // apply turn deadband
    if(-Constants.TURN_DEADBAND < turn && turn < Constants.TURN_DEADBAND) {
      turn = 0.0;
    } // end if      

    // apply sensitivity
    forward  = sensitivity(forward,  Constants.FORWARD_SENSITIVITY);
    turn = sensitivity(turn, Constants.TURN_SENSITIVITY);
    
    // driveTrain.setSpeed(forward, turn);
    
    double driveLeftPower =  forward - (turn * turnThrottle); //EW 20 sep 2022
    double driveRightPower = forward + (turn * turnThrottle); //EW 20 sep 2022

    // *** start of changes PG 31 May 2022 ***  
      
    // apply throttle
    driveLeftPower  = driveLeftPower  * throttle; //EW 20 sep 2022
    driveRightPower = driveRightPower * throttle; //EW 20 sep 2022

    //apply smoothing
    //driveLeftPower  = applyRampBand(driveLeftPower);
    //driveRightPower = applyRampBand(driveRightPower);

    // *** end of changes PG 31 May 2022 ***

    driveLeftA. set(driveLeftPower);  // tested, works ok,   PG 06 Oct 2022
    driveLeftB. set(driveLeftPower);  // tested, works ok,   PG 06 Oct 2022
    driveRightA.set(driveRightPower); // tested, works ok,   PG 31 May 2022
    driveRightB.set(driveRightPower); // tested, works ok,   PG 31 May 2022

    //Intake controls
    //Control Intake/Shooting
    if(operatorController.getXButton()){ //EW 20 sep 2022
      intake.set(Constants.SHOOTER_SPEED); // DONE: check if change is needed
    } // end if
    else if(operatorController.getYButton()){ // EW 20 sep 2022
      intake.set(Constants.INTAKE_SPEED); // DONE: check if change is needed
    } // end else if
    else{
      intake.set(0.0); // DONE: check if change is needed
    } // end else

  operateArm();

  reportStatus();

  } // end teleopPeriodic()

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    driveLeftA.   set(0.0);
    driveLeftB.   set(0.0);
    driveRightA.  set(0.0);
    driveRightB.  set(0.0);
    arm.          set(0.0);

    intake.       set(0.0);  // DONE: check if change is needed

    reportStatus();

  } // end disabledInit();
    
// ------------------------------------------------------------------------------------

  // *** start of changes PG 03 JUN 2022 ***

  // arm states

  private static final int ARM_IS_UP                 = 1;
  private static final int ARM_GOING_DOWN            = 2;
  private static final int ARM_IS_DOWN               = 3;
  private static final int ARM_GOING_UP              = 4;
  private static final int ARM_EMERGENCY_STOP        = 5;
  private static final int ARM_CANCEL_EMERGENCY_STOP = 6;

  private int state = ARM_IS_UP; // initial arm state is UP at the start of a match

  private void operateArm() {

    if (operatorController.getBButton()) state = ARM_EMERGENCY_STOP;

    // arm actions implemented as a finite-state machine 

    switch (state) {

       case ARM_IS_UP:
        arm.set(Constants.ARM_HOLD_UP);
        SmartDashboard.putString("Arm Status", "IS UP");
        if (operatorController.getStartButton())                        state = ARM_GOING_DOWN;
      break; // end case ARM_IS_UP
      
      case ARM_GOING_DOWN:
        arm.set(Constants.ARM_TRAVEL_DOWN);
        SmartDashboard.putString("Arm Status", "GOING DOWN");
        if (downLimitSwitch.get())                                      state = ARM_IS_DOWN;
        if (arm.getEncoder().getPosition() <= Constants.ARM_DOWN_LIMIT) state = ARM_IS_DOWN;
      break; // end case ARM_GOING_DOWN
    
      case ARM_IS_DOWN: 
        arm.set(Constants.ARM_HOLD_DOWN);
        SmartDashboard.putString("Arm Status", "DOWN");
        if (operatorController.getBackButton())                         state = ARM_GOING_UP; 
      break; // end case ARM_IS_DOWN

      case ARM_GOING_UP: 
        arm.set(Constants.ARM_TRAVEL_UP);
        SmartDashboard.putString("Arm Status", "GOING UP");
        if (upLimitSwitch.get())                                        state = ARM_IS_UP; 
        if (arm.getEncoder().getPosition() >= Constants.ARM_UP_LIMIT)   state = ARM_IS_UP;
      break; // end case ARM_GOING_UP

      case ARM_EMERGENCY_STOP:
        arm.set(0.0);
        SmartDashboard.putString("Arm Status", "EMERGENCY STOP");
        if (operatorController.getAButton())                            state = ARM_CANCEL_EMERGENCY_STOP; 
      break; // end case ARM_EMERGENCY_STOP
    
      case ARM_CANCEL_EMERGENCY_STOP:
        // arm is stopped so choose next state
        SmartDashboard.putString("Arm Status", "CANCELLED STOP");
        if      (operatorController.getStartButton())                   state = ARM_GOING_UP;
        else if (operatorController.getBackButton())                    state = ARM_GOING_DOWN;
      break; // end case ARM_EMERGENCY_STOP

    } // end switch (state)

    reportStatus();

  } // end operateArm()

// *** end of changes PG 03 JUN 2022 ***

// *** start of changes PG 31 May 2022 ***

private double sensitivity (double speed, int howSensitive) {
  switch (howSensitive){
      case Constants.LINEAR:    return speed;
      case Constants.QUADRATIC: return Math.signum(speed) * Math.pow(speed,2.0);
      case Constants.CUBIC:     return                      Math.pow(speed,3.0);
      case Constants.QUARTIC:   return Math.signum(speed) * Math.pow(speed,4.0);
      default:                  return 0.0;
  } // end switch how sensitive
} // end sensitivity()

private double applyRampBand(double motorSpeed) {
  if (motorSpeed - previousMotorSpeed < -rampBand) {
    motorSpeed = previousMotorSpeed - rampBand;
  } // end if 
  else if (motorSpeed - previousMotorSpeed > rampBand) {
    motorSpeed = previousMotorSpeed + rampBand;
  } // end else if

  previousMotorSpeed = motorSpeed; 
  return motorSpeed;
} // end applyRampBand ()

private void reportStatus() {
  SmartDashboard.putBoolean("Go For Auto", goForAuto);

  SmartDashboard.putNumber("Left  A Temp", driveLeftA.getMotorTemperature());
  SmartDashboard.putNumber("Left  B Temp", driveLeftB.getMotorTemperature());
  SmartDashboard.putNumber("Right A Temp", driveRightA.getMotorTemperature());
  SmartDashboard.putNumber("Right B Temp", driveRightB.getMotorTemperature());
  SmartDashboard.putNumber("Arm Temp",     arm.getMotorTemperature());

  SmartDashboard.putNumber("Left  A Pos",   encoderLeftA.getPosition());
  // SmartDashboard.putNumber("Left  B EV",   encoderLeftB.getPosition());
  SmartDashboard.putNumber("Right A Pos",   encoderRightA.getPosition());
  // SmartDashboard.putNumber("Right B EV",   encoderRightB.getPosition());
  SmartDashboard.putNumber("Arm Position", encoderArm.getPosition());

  SmartDashboard.putNumber("Left  A RPM", encoderLeftA.getVelocity());
  // SmartDashboard.putNumber("Left  B RPM", encoderLeftB.getVelocity());
  SmartDashboard.putNumber("Right A RPM", encoderRightA.getVelocity());
  // SmartDashboard.putNumber("Right B RPM", encoderRightB.getVelocity());
  SmartDashboard.putNumber("Arm RPM",     encoderArm.getVelocity());

  // E.W. 19 SEP 2022
  SmartDashboard.putNumber("Throttle",    driverController.getRawAxis(3));

  SmartDashboard.putBoolean("Arm up?",             upLimitSwitch.get());
  SmartDashboard.putBoolean("Arm down?",           downLimitSwitch.get());
  SmartDashboard.putBoolean("Arm going up?",       isMovingUp);
  SmartDashboard.putBoolean("Arm going down?",     isMovingDown);
  SmartDashboard.putBoolean("Arm Emergency stop?", ! isArmStopped);
  
  SmartDashboard.putNumber("Initial Arm Position",initialArmPosition);

  SmartDashboard.putNumber("Left A A",  pdp.getCurrent(Constants.PDP_LEFT_A));
  SmartDashboard.putNumber("Left B A",  pdp.getCurrent(Constants.PDP_LEFT_B));
  SmartDashboard.putNumber("Right A A", pdp.getCurrent(Constants.PDP_RIGHT_A));
  SmartDashboard.putNumber("Right B A", pdp.getCurrent(Constants.PDP_RIGHT_B));
  SmartDashboard.putNumber("Arm A",     pdp.getCurrent(Constants.PDP_ARM));
  SmartDashboard.putNumber("Shooter A", pdp.getCurrent(Constants.PDP_SHOOTER));

  SmartDashboard.putNumber("PDP Volts A V", pdp.getVoltage());
  SmartDashboard.putNumber("PDP Power A W", pdp.getTotalPower());



} // end reportStatus();

// *** end of changes PG 31 May 2022 ***


} // end class Robot