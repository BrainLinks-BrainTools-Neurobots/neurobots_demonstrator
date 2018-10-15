package com.kuka.roboticsAPI.smartServo.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.ServoMotion;
import com.kuka.roboticsAPI.motionModel.SmartServo;

/**
 * This example activates a SmartServo motion in position control mode, sends a
 * sequence of joint specific set points, describing a sine function and
 * evaluates the statistic timing.
 */
public class SmartServoSampleSimpleJointMotion extends RoboticsAPIApplication
{

    private LBR lbr;
    private ISmartServoRuntime theSmartServoRuntime = null;

    // Tool Data
    private Tool toolAttachedToLBR;
    private LoadData loadData;
    private final String toolFrame = "toolFrame";
    private final double[] translationOfTool = { 0, 0, 100 };
    private final double mass = 0;
    private final double[] centerOfMassInMillimeter = { 0, 0, 100 };

    private int count = 0;

    private final int milliSleepToEmulateComputationalEffort = 30;
    private final int numRuns = 1000;
    private final double amplitude = 0.2;
    private final double freqency = 0.1;
    private int steps = 0;

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

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is
     * collision free.
     */
    public void moveToInitialPosition()
    {
        toolAttachedToLBR.move(
                ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        /*
         * 
         * Note: The Validation itself justifies, that in this very time
         * instance, the load parameter setting was sufficient. This does not
         * mean by far, that the parameter setting is valid in the sequel or
         * lifetime of this program
         */
        try
        {
            if (!ServoMotion.validateForImpedanceMode(toolAttachedToLBR))
            {
                getLogger().info("Validation of torque model failed - correct your mass property settings");
                getLogger().info("SmartServo will be available for position controlled mode only, until validation is performed");
            }
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }

    @Override
    public void run()
    {

        moveToInitialPosition();

        boolean doDebugPrints = false;

        JointPosition initialPosition = new JointPosition(
                lbr.getCurrentJointPosition());
        SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 20% of systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.2);
        aSmartServoMotion.setJointVelocityRel(0.2);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting SmartServo motion in position control mode");
        toolAttachedToLBR.getDefaultMotionFrame().moveAsync(aSmartServoMotion);

        getLogger().info("Get the runtime of the SmartServo motion");
        theSmartServoRuntime = aSmartServoMotion.getRuntime();

        // create an JointPosition Instance, to play with
        JointPosition destination = new JointPosition(
                lbr.getJointCount());
        getLogger().info("Start loop");
        // For Roundtrip time measurement...
        StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            double omega = freqency * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
            for (steps = 0; steps < numRuns; ++steps)
            {
                // Timing - draw one step
                OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(milliSleepToEmulateComputationalEffort);
                theSmartServoRuntime.updateWithRealtimeSystem();
                // Get the measured position
                JointPosition curMsrJntPose = theSmartServoRuntime
                        .getAxisQMsrOnController();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                    destination.set(k, Math.sin(sinArgument)
                            * amplitude + initialPosition.get(k));
                    if (k > 5)
                    {
                        destination.set(k, initialPosition.get(k));
                    }
                }
                theSmartServoRuntime.setDestination(destination);

                // ////////////////////////////////////////////////////////////
                if (doDebugPrints)
                {
                    getLogger().info("Step " + steps + " New Goal "
                            + destination);
                    getLogger().info("Fine ipo finished " + theSmartServoRuntime.isDestinationReached());
                    if (theSmartServoRuntime.isDestinationReached())
                    {
                        count++;
                    }
                    getLogger().info("Ipo state " + theSmartServoRuntime.getFineIpoState());
                    getLogger().info("Remaining time " + theSmartServoRuntime.getRemainingTime());
                    getLogger().info("LBR Position "
                            + lbr.getCurrentJointPosition());
                    getLogger().info(" Measured LBR Position "
                            + curMsrJntPose);
                    if (steps % 100 == 0)
                    {
                        // Some internal values, which can be displayed
                        getLogger().info("Simple Joint Test - step " + steps + theSmartServoRuntime.toString());

                    }
                }
                aStep.end();

            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }
        ThreadUtil.milliSleep(1000);

        //Print statistics and parameters of the motion
        getLogger().info("Displaying final states after loop ");
        getLogger().info(getClass().getName() + theSmartServoRuntime.toString());
        // Stop the motion

        getLogger().info("Stop the SmartServo motion");
        theSmartServoRuntime.stopMotion();

        getLogger().info(count + " times was the destination reached.");
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("Under Windows, you should play with the registry, see the e.g. the SmartServo Class javaDoc for details");
        }
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(String[] args)
    {
        SmartServoSampleSimpleJointMotion app = new
                SmartServoSampleSimpleJointMotion();
        app.runApplication();
    }
}
