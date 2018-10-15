package com.kuka.roboticsAPI.directServo.examples;

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
import com.kuka.roboticsAPI.motionModel.DirectServo;
import com.kuka.roboticsAPI.motionModel.IDirectServoRuntime;
import com.kuka.roboticsAPI.motionModel.ServoMotion;

/**
 * This example activates a DirectServo motion in position control mode, sends a
 * sequence of joint specific set points, describing a sine function and
 * evaluates the statistic timing.
 */
public class DirectServoSampleSimpleJointMotion extends RoboticsAPIApplication
{
    private LBR lbr;
    private IDirectServoRuntime theDirectServoRuntime = null;

    // Tool Data
    private Tool toolAttachedToLBR;
    private LoadData loadData;

    // Tool Data
    private final String toolFrame = "toolFrame";
    private final double[] translationOfTool = { 0, 0, 100 };
    private final double mass = 0;
    private final double[] centerOfMassInMillimeter = { 0, 0, 100 };

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
     * collision free
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
                getLogger().info("DirectServo will be available for position controlled mode only, until validation is performed");
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

        JointPosition initialPosition = new JointPosition(
                lbr.getCurrentJointPosition());
        DirectServo aDirectServoMotion = new DirectServo(initialPosition);

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        toolAttachedToLBR.getDefaultMotionFrame().moveAsync(aDirectServoMotion);

        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime exists AFTER motion command was issued
        theDirectServoRuntime = aDirectServoMotion
                .getRuntime();

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
                theDirectServoRuntime.updateWithRealtimeSystem();

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
                theDirectServoRuntime
                        .setDestination(destination);

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
        getLogger().info(getClass().getName() + " \n" + theDirectServoRuntime.toString());
        // Stop the motion
        getLogger().info("Stop the SmartServo motion");
        theDirectServoRuntime.stopMotion();
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("Under Windows, you should play with the registry, see the e.g. the SmartServo Class javaDoc for details");
        }
    }

    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        DirectServoSampleSimpleJointMotion app = new DirectServoSampleSimpleJointMotion();
        app.runApplication();

    }
}
