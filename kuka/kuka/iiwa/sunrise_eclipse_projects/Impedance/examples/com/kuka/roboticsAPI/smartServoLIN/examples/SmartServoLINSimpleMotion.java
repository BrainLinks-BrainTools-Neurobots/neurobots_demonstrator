package com.kuka.roboticsAPI.smartServoLIN.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.ISmartServoLINRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServoLIN;

/**
 * This sample activates a SmartServoLIN motion in position control mode, sends
 * a sequence of Cartesian set points, describing a sine function in z-direction
 * and evaluates the statistic timing.
 */
public class SmartServoLINSimpleMotion extends RoboticsAPIApplication
{
    private LBR lbr;
    private Tool toolAttachedToLBR;
    private LoadData loadData;
    private ISmartServoLINRuntime smartServoLINRuntime = null;

    // Tool Data
    private final String toolFrame = "toolFrame";
    private final double[] translationOfTool = { 0, 0, 100 };
    private final double mass = 0;
    private final double[] centerOfMassInMillimeter = { 0, 0, 100 };

    private final int numRuns = 600;
    private final double amplitude = 70;
    private final double freqency = 0.6;

    private static final int milliSleepToEmulateComputationalEffort = 30;

    @Override
    public void initialize()
    {
        lbr = getContext().getDeviceFromType(LBR.class);

        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        loadData = new LoadData();
        loadData.setMass(mass);
        loadData.setCenterOfMass(
                centerOfMassInMillimeter[0], centerOfMassInMillimeter[1],
                centerOfMassInMillimeter[2]);
        toolAttachedToLBR = new Tool("Tool", loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                translationOfTool[0], translationOfTool[1],
                translationOfTool[2]);
        ObjectFrame aTransformation = toolAttachedToLBR.addChildFrame(toolFrame
                + "(TCP)", trans);
        toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        toolAttachedToLBR.attachTo(lbr.getFlange());
    }

    @Override
    public void run()
    {
        getLogger().info("Move to start position.");
        lbr.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));

        AbstractFrame initialPosition = lbr.getCurrentCartesianPosition(lbr
                .getFlange());

        // Create a new smart servo linear motion
        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServoLIN in position control mode");
        lbr.getFlange().moveAsync(aSmartServoLINMotion);

        getLogger().info("Get the runtime of the SmartServoLIN motion");
        smartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        StatisticTimer timing = new StatisticTimer();

        // Start the smart servo lin sine movement
        timing = startSineMovement(smartServoLINRuntime, timing);

        ThreadUtil.milliSleep(1000);

        getLogger().info("Print statistic timing");
        getLogger()
                .info(getClass().getName() + smartServoLINRuntime.toString());

        getLogger().info("Stop the SmartServoLIN motion");
        smartServoLINRuntime.stopMotion();

        // Statistic Timing of sine movement loop
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger()
                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger()
                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
        }
    }

    private StatisticTimer startSineMovement(
            ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
    {
        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(lbr
                .getFlange());

        getLogger().info("Do sine movement");
        try
        {
            double omega = freqency * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();

            for (int i = 0; i < numRuns; ++i)
            {
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system

                ThreadUtil.milliSleep(milliSleepToEmulateComputationalEffort);

                // Update the smart servo LIN runtime
                smartServoLINRuntime.updateWithRealtimeSystem();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // Compute the sine function
                Frame destFrame = new Frame(aFrame);
                destFrame.setZ(amplitude * Math.sin(sinArgument));

                // Set new destination
                smartServoLINRuntime.setDestination(destFrame);
                aStep.end();
            }

        }
        catch (Exception e)
        {
            getLogger().error(e.getLocalizedMessage());
            e.printStackTrace();
        }
        return timing;
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(String[] args)
    {
        SmartServoLINSimpleMotion app = new SmartServoLINSimpleMotion();
        app.runApplication();
    }

}
