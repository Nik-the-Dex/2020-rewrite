package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SensorInputs.NavX;
import frc.robot.subsystems.BallMech;
import frc.robot.subsystems.ClimbMech;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot
{
    public static Drivetrain drivetrain;
    public static NavX       navX;
    public static ColorWheel wheelOfFortune;
    public static BallMech   ballMech;
    public static ClimbMech  climbMech;

    public static CurrentLimitsConfigs currentLimit;

    private final Compressor compressor1 = new Compressor(Constants.PCM1, PneumaticsModuleType.CTREPCM);
    private final Compressor compressor2 = new Compressor(Constants.PCM2, PneumaticsModuleType.CTREPCM);

    @Override
    public void robotInit()
    {
        compressor1.enableDigital();
        compressor2.enableDigital();

        currentLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentLowerLimit(20)
            .withSupplyCurrentLowerTime(1);

        navX           = new NavX();
        drivetrain     = new Drivetrain();
        wheelOfFortune = new ColorWheel();
        ballMech       = new BallMech();
        climbMech      = new ClimbMech();

        climbMech.hangerSolenoid.set(Value.kReverse);
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        navX.updateAHRS();
        IO.update();
    }

    @Override
    public void autonomousInit()
    {
        navX.resetAngle();
        drivetrain.autoState        = 0;
        drivetrain.autoCounter      = 0;
        drivetrain.driveStraightState = 0;
    }

    @Override
    public void autonomousPeriodic()
    {
        drivetrain.sixBallAutoTwo();
    }

    @Override
    public void teleopInit()
    {
        drivetrain.rightTalon1.getConfigurator().apply(currentLimit);
        drivetrain.rightTalon2.getConfigurator().apply(currentLimit);
        drivetrain.leftTalon1.getConfigurator().apply(currentLimit);
        drivetrain.leftTalon3.getConfigurator().apply(currentLimit);

        navX.resetAngle();
    }

    @Override
    public void teleopPeriodic()
    {
        IO.updateControllers();
        drivetrain.sigmaDrive(IO.m_leftAnalogY, IO.m_rightAnalogY);
        IO.processControllers();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}
