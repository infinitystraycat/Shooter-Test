package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Autonomous.CatzAutonomous;
import frc.Mechanisms.CatzShooter;
import frc.Mechanisms.CatzElevator;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzDriveTrain;


public class Robot extends TimedRobot 
{
  public static CatzShooter shooter;
  public static CatzElevator elevator;
  public static CatzIntake intake;
  public static CatzDriveTrain driveTrain;

  public static CatzAutonomous auton;
  public static Timer autonomousTimer;

  public static DataCollection dataCollection;
  public static CatzLog        catzLog;
  public static Timer dataCollectionTimer;
  public ArrayList<CatzLog> dataArrayList;

  public static PowerDistributionPanel pdp;

  public static AHRS navx;
  
//-------------------------------------------Drive Controller------------------------------------------------------------- 
  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;
  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;

//-------------------------------------------Elevator------------------------------------------------------------- 
  public final double ELEV_POWER = 0.5;
  public final double ELEV_FACTOR = 1.3;

//-------------------------------------------Path Chooser------------------------------------------------------------- 
  
  
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  @Override
  public void robotInit() 
  {
    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    driveTrain = new CatzDriveTrain();
    intake = new CatzIntake();
    auton = new CatzAutonomous();
    elevator = new CatzElevator();
    shooter = new CatzShooter();

    dataArrayList = new ArrayList<CatzLog>();
    
    dataCollection = new DataCollection();
  
    navx = new AHRS(Port.kMXP, (byte)200);
    navx.reset();
    
    pdp = new PowerDistributionPanel();
    
    autonomousTimer     = new Timer();
    dataCollectionTimer = new Timer();

    dataCollection.dataCollectionInit(dataArrayList);
    dataCollectionTimer.reset();
    dataCollectionTimer.start();
    dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_STRAIGHT);
    //dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_TURN);
    dataCollection.startDataCollection();

    intake.intakeControl();
    
  }

  @Override

  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("RPM", shooter.getRPM());
    SmartDashboard.putNumber("Rotation", navx.getAngle());
  }

  @Override
  public void autonomousInit() 
  {
    shooter.inAutonomous = true;
    auton.driveStraight(120,10,100);
    shooter.setTargetRPM(4500);
    //auton.driveStraight(120, 14, 100);
    //auton.TurnInPlace(90, 1000);
  }

  @Override
  public void autonomousPeriodic() 
  {


  }


  @Override
  public void teleopInit() 
  {
    driveTrain.instantiateDifferentialDrive();
  }

  @Override
  public void teleopPeriodic() 
  {
    //-------------------------------------------Drivetrain------------------------------------------------------------- 
    driveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), xboxDrv.getX(Hand.kRight));
  
    if(xboxDrv.getBumper(Hand.kLeft))
    {
      driveTrain.shiftToHighGear();
    }
    else if(xboxDrv.getBumper(Hand.kRight))
    {
      driveTrain.shiftToLowGear();
    }
    //-------------------------------------------Intake------------------------------------------------------------- 
    if(xboxDrv.getStickButtonPressed(Hand.kLeft))
    {
      intake.deployIntake(); 
    }
    else if(xboxDrv.getStickButtonPressed(Hand.kRight))
    {
      intake.stowIntake();
    }
    
    if(xboxDrv.getTriggerAxis(Hand.kLeft) > 0.2)//roller out
    {
      intake.intakeRollerIn();                                                                                                                                                                                                                      
    }
    else if(xboxDrv.getTriggerAxis(Hand.kRight) > 0.2)//roller in
    {
      intake.intakeRollerOut();
    }
    else
    {
      intake.intakeRollerOff();
    }
    //-----------------------shooter-----------------------
    
     if(xboxAux.getPOV() == DPAD_DN)
    {
      shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_HI);
    }
    else if(xboxAux.getBButton())
    {
      //indexer.setShooterIsRunning(true);
      shooter.shoot();
    } 
    else if(xboxAux.getStartButton())
    {
      shooter.shooterOff();
    } 
  }

  @Override
  public void disabledInit() 
  {
    dataCollection.stopDataCollection();
    Robot.driveTrain.drvTrainMtrCtrlLTFrnt.setNeutralMode(NeutralMode.Coast); 
    Robot.driveTrain.drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Coast);
    Robot.driveTrain.drvTrainMtrCtrlRTFrnt.setNeutralMode(NeutralMode.Coast);
    Robot.driveTrain.drvTrainMtrCtrlRTBack.setNeutralMode(NeutralMode.Coast);

    try 
    {
      dataCollection.exportData(dataArrayList);
    } catch (Exception e) 
    {
      e.printStackTrace();
    }

  }

  @Override
  public void disabledPeriodic() 
  {
  }

  @Override
  public void testInit() 
  {
  }

  @Override
  public void testPeriodic() 
  {
  }
}