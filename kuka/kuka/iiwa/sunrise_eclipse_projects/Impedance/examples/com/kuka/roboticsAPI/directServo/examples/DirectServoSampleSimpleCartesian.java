package com.kuka.roboticsAPI.directServo.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.DirectServo;
import com.kuka.roboticsAPI.motionModel.IDirectServoRuntime;
import com.kuka.roboticsAPI.motionModel.ServoMotion;

/**
 * This example activates a DirectServo motion in position control mode, sends a
 * sequence of Cartesian set points, describing a sine function and evaluates
 * the statistic timing.
 */
public class DirectServoSampleSimpleCartesian extends RoboticsAPIApplication
{
    private LBR lbr;

    // Tool Data
    private Tool toolAttachedToLBR;
    private LoadData loadData;
    private final String toolFrame = "toolFrame";
    private final double[] translationOfTool = { 0, 0, 100 };
    private final double mass = 0;
    private final double[] centerOfMassInMillimeter = { 0, 0, 100 };

    private final int milliSleepToEmulateComputationalEffort = 30;
    private final int numRuns = 1000;
    private final double amplitude = 70;
    private final double freqency = 0.1;

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
    private void moveToInitialPosition()
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

    public void moveToSomePosition()
    {
        toolAttachedToLBR.move(
                ptp(0., Math.PI / 180 * 20., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.));
    }

    /**
     * Main Application Routine
     */
    @Override
    public void run()
    {
        moveToInitialPosition();

        boolean doDebugPrints = false;

        DirectServo aDirectServoMotion = new DirectServo(
                lbr.getCurrentJointPosition());

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        toolAttachedToLBR.moveAsync(aDirectServoMotion);

        getLogger().info("Get the runtime of the DirectServo motion");
        IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion
                .getRuntime();

        Frame aFrame = theDirectServoRuntime.getCurrentCartesianDestination(toolAttachedToLBR.getDefaultMotionFrame());

        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec
            double omega = freqency * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
            for (int i = 0; i < numRuns; ++i)
            {

                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
                theDirectServoRuntime.updateWithRealtimeSystem();
                // Get the measured position in Cartesian...
                Frame msrPose = theDirectServoRuntime
                        .getCurrentCartesianDestination(toolAttachedToLBR.getDefaultMotionFrame());

                if (doDebugPrints)
                {
                    getLogger().info("Current cartesian goal " + aFrame);
                    getLogger().info("Current joint destination "
                            + theDirectServoRuntime.getCurrentJointDestination());
                }

                // Do some Computation
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(milliSleepToEmulateComputationalEffort);

                // do a cyclic loop
                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // compute a new commanded position
                Frame destFrame = aFrame.copyWithRedundancy();
                double offset = amplitude * Math.sin(sinArgument);
                destFrame.setZ(destFrame.getZ() + offset);

                if (doDebugPrints)
                {
                    getLogger().info("New cartesian goal " + destFrame);
                    getLogger().info("LBR position "
                            + lbr.getCurrentCartesianPosition(lbr
                                    .getFlange()));
                    getLogger().info("Measured cartesian pose from runtime "
                            + msrPose);
                    if ((i % 100) == 0)
                    {
                        getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());
                    }
                }

                theDirectServoRuntime.setDestination(destFrame);

            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }

        //Print statistics and parameters of the motion
        getLogger().info("Simple Cartesian Test \n" + theDirectServoRuntime.toString());

        getLogger().info("Stop the DirectServo motion");
        theDirectServoRuntime.stopMotion();
    }

    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        DirectServoSampleSimpleCartesian app = new DirectServoSampleSimpleCartesian();
        app.runApplication();
    }
}
