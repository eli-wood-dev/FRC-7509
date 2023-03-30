/*
  2023 rbot code
  written by Eli Wood
*/

package frc.robot;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; // DONE: check if change is needed (comment this out?)

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.*;

public class Robot extends TimedRobot {
  //Definitions for the hardware. Change this if you change what stuff you have plugged in
  //Mostly adjusted to correct Can IDs
  TalonFX driveLeftA  = new TalonFX(Constants.CAN_LEFT_A);
  TalonFX driveLeftB  = new TalonFX(Constants.CAN_LEFT_B);
  TalonFX driveRightA = new TalonFX(Constants.CAN_RIGHT_A);
  TalonFX driveRightB = new TalonFX(Constants.CAN_RIGHT_B);

  CANSparkMax rotate = new CANSparkMax(Constants.CAN_ROTATE, MotorType.kBrushless);
  CANSparkMax gripper = new CANSparkMax(Constants.CAN_GRIPPER, MotorType.kBrushless);

  TalonFX vertical = new TalonFX(Constants.CAN_VERTICAL);
  TalonFX extend = new TalonFX(Constants.CAN_EXTEND);

  Arm arm;
  
  Joystick driverController         = new Joystick(Constants.USB_DRIVER_CONTROLLER);
  XboxController operatorController = new XboxController(Constants.USB_OPERATOR_CONTROLLER);


  //DigitalInput downLimitSwitch = new DigitalInput(0);

  RelativeEncoder encoderRotate = rotate.getEncoder();

  PowerDistribution pdp = new PowerDistribution();

  //Varibles needed for the code

  double autoStart = 0;
  boolean goForAuto = false;

  //Speed ramping
  private static double rampBand = Constants.DEFAULT_RAMP_BAND; //EW 21 sep 2022
  private static double previousMotorSpeed = 0.0; //EW 21 sep 2022

  boolean armOut = true;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    //Configure motors to turn correct direction. You may have to invert some of your motors

    rotate.restoreFactoryDefaults();
    rotate.clearFaults();
    rotate.setIdleMode(IdleMode.kBrake);

    gripper.restoreFactoryDefaults();
    gripper.clearFaults();
    gripper.setIdleMode(IdleMode.kBrake);

    driveLeftA. setInverted(true);
    driveLeftB. setInverted(true);
    driveRightA.setInverted(false);
    driveRightB.setInverted(false);
    extend.setInverted(false);
    vertical.setInverted(false);
    rotate.setInverted(false);

    rotate.burnFlash();

    gripper.setInverted(false);

    gripper.burnFlash();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    
    arm = new Arm(rotate, gripper, vertical, extend);


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
    
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if(goForAuto){
      
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
    
    
    //Set up arcade steer
    double forward = driverController.getRawAxis(1); // y axis id negative forwards
    double turn    = driverController.getRawAxis(0);  // x axis

    // define throttle
    // EW 20 Sep 2022 
    
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

    driveLeftA. set(ControlMode.PercentOutput, driveLeftPower);  
    driveLeftB. set(ControlMode.PercentOutput, driveLeftPower); 
    driveRightA.set(ControlMode.PercentOutput, driveRightPower);
    driveRightB.set(ControlMode.PercentOutput, driveRightPower); 

    //control the arm

    arm.rotate(operatorController.getRawAxis(Constants.RX)); //change to desired axis
    arm.vRotate(operatorController.getRawAxis(Constants.RY));

    if(operatorController.getRawButton(Constants.X_BUTTON)){
      if(armOut){
        arm.retract(-1);
      } else{
        arm.extend();
      }
      armOut = !armOut;
    }

    if(operatorController.getRawButton(Constants.A_BUTTON)){
      arm.open();
    }

    if(operatorController.getRawButton(Constants.B_BUTTON)){
      arm.close();
    }
    

  reportStatus();

  } // end teleopPeriodic()

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    driveLeftA.   set(ControlMode.PercentOutput, 0.0);
    driveLeftB.   set(ControlMode.PercentOutput, 0.0);
    driveRightA.  set(ControlMode.PercentOutput, 0.0);
    driveRightB.  set(ControlMode.PercentOutput, 0.0);

    rotate.set(0);
    gripper.set(0);
    extend.set(ControlMode.PercentOutput, 0.0);
    vertical.set(ControlMode.PercentOutput, 0.0);

    reportStatus();

  } // end disabledInit();

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

  // E.W. 19 SEP 2022
  SmartDashboard.putNumber("Throttle",    driverController.getRawAxis(3));
  
  SmartDashboard.putNumber("Left A A",  pdp.getCurrent(Constants.PDP_LEFT_A));
  SmartDashboard.putNumber("Left B A",  pdp.getCurrent(Constants.PDP_LEFT_B));
  SmartDashboard.putNumber("Right A A", pdp.getCurrent(Constants.PDP_RIGHT_A));
  SmartDashboard.putNumber("Right B A", pdp.getCurrent(Constants.PDP_RIGHT_B));

  SmartDashboard.putNumber("PDP Volts A V", pdp.getVoltage());
  SmartDashboard.putNumber("PDP Power A W", pdp.getTotalPower());



} // end reportStatus();

// *** end of changes PG 31 May 2022 ***


} // end class Robot