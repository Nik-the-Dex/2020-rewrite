package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class IO
{
    public static final XboxController mainController     = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
    public static final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    public static boolean m_buttonA;
    public static boolean m_buttonB;
    public static boolean m_buttonX;
    public static boolean m_buttonY;
    public static boolean m_leftBumper;
    public static boolean m_leftBumperReleased;
    public static boolean m_rightBumper;
    public static boolean m_leftStick;
    public static boolean m_rightStick;
    public static boolean m_pauseButton;
    public static boolean m_settingsButton;

    public static double m_leftTrigger;
    public static double m_rightTrigger;
    public static double m_leftAnalogX;
    public static double m_rightAnalogX;
    public static double m_leftAnalogY;
    public static double m_rightAnalogY;
    public static int    m_DPad;

    public static boolean wofRunning = false;

    private static boolean hangerPulseActive = false;
    private static int     hangerPulseCounter = 0;

    private IO() {}

    public static void updateControllers()
    {
        m_buttonA            = mainController.getAButton();
        m_buttonB            = mainController.getBButton();
        m_buttonX            = mainController.getXButton();
        m_buttonY            = mainController.getYButtonPressed();
        m_leftBumper         = mainController.getLeftBumper();
        m_leftBumperReleased = mainController.getLeftBumperReleased();
        m_rightBumper        = mainController.getRightBumper();
        m_leftStick          = mainController.getLeftStickButton();
        m_rightStick         = mainController.getRightStickButton();
        m_leftTrigger        = mainController.getLeftTriggerAxis();
        m_rightTrigger       = mainController.getRightTriggerAxis();
        m_leftAnalogX        = mainController.getLeftX();
        m_rightAnalogX       = mainController.getRightX();
        m_leftAnalogY        = mainController.getLeftY();
        m_rightAnalogY       = mainController.getRightY();
        m_DPad               = mainController.getPOV();
        m_pauseButton        = mainController.getStartButtonPressed();
        m_settingsButton     = mainController.getBackButtonPressed();
    }

    public static void update()
    {
        SmartDashboard.putNumber("leftanalog",  m_leftAnalogY);
        SmartDashboard.putNumber("rightanalog", m_rightAnalogY);
    }

    public static void processControllers()
    {
        if (m_rightTrigger > 0.5)
        {
            Robot.ballMech.variableDistanceShooter();
        }
        else
        {
            Robot.ballMech.stopShooter();
            Robot.ballMech.runRoller(0);
            Robot.ballMech.counter      = 0;
            Robot.ballMech.shooterState = 0;
        }

        if (m_leftTrigger > 0.5)
        {
            Robot.drivetrain.highGear(true);
        }
        else
        {
            Robot.drivetrain.highGear(false);
        }

        if (m_leftBumper)
        {
            Robot.ballMech.runRollerWithIntake();
            Robot.ballMech.intake(-1);
        }
        else if (m_rightBumper)
        {
            Robot.ballMech.runRollerWithIntake();
            Robot.ballMech.intake(1);
        }
        else if (!(m_rightTrigger > 0.5))
        {
            Robot.ballMech.stopIntake();
            Robot.ballMech.rollerState = 0;
        }

        if (m_buttonY)
        {
            Robot.wheelOfFortune.toggleWOFCylinder();
        }

        if (m_buttonA)
        {
            wofRunning = true;
        }

        if (wofRunning)
        {
            Robot.wheelOfFortune.runWOF();
        }
        else
        {
            mainController.setRumble(RumbleType.kLeftRumble, 0);
            Robot.wheelOfFortune.WOFmotor.set(0);
            Robot.wheelOfFortune.WOFencoder.setPosition(0);
        }

        if (m_pauseButton)
        {
            Robot.climbMech.extendHanger();
            Robot.climbMech.weirdClimbMotor.set(0.5);
            hangerPulseActive  = true;
            hangerPulseCounter = 0;
        }

        if (hangerPulseActive)
        {
            hangerPulseCounter++;
            if (hangerPulseCounter > 46)
            {
                Robot.climbMech.weirdClimbMotor.set(0);
                hangerPulseActive  = false;
                hangerPulseCounter = 0;
            }
        }

        if (m_settingsButton)
        {
            Robot.climbMech.hangerSolenoid.set(Value.kReverse);
        }

        if (m_DPad == 0)
        {
            Robot.climbMech.climb();
        }
        else if (m_DPad == 180)
        {
            Robot.climbMech.setClimbMotors(-0.3);
        }
        else
        {
            Robot.climbMech.setClimbMotors(0);
        }
    }
}
