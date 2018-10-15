package com.kuka.roboticsAPI.smartServoLIN.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.ISmartServoLINRuntime;
import com.kuka.roboticsAPI.motionModel.ServoMotion;
import com.kuka.roboticsAPI.motionModel.SmartServoLIN;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * This example activates a SmartServoLIN motion in Cartesian impedance control
 * mode, sends a sequence of Cartesian set points, describing a sine function in
 * z-direction and modifies compliance parameters during the motion.
 * 
 */
public class SmartServoLINInteractionControl extends RoboticsAPIApplication
{

    private LBR lbr;
    private Tool toolAttachedToLBR;
    private LoadData loadData;
    private ISmartServoLINRuntime theSmartServoLINRuntime = null;

    // Tool Data
    private final String toolFrame = "toolFrame";
    private final double[] translationOfTool = { 0, 0, 100 };
    private final double mass = 0;
    private final double[] centerOfMassInMillimeter = { 0, 0, 100 };

    private final int numRuns = 600;
    private final double amplitude = 70;
    private final double freqency = 0.6;

    private final double[] maxTranslationVelocity = { 150, 150, 150 };
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

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is
     * collision free.
     */
    public void moveToInitialPosition()
    {
        lbr.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        /*
         * Note: The Validation itself justifies, that in this very time
         * instance, the load parameter setting was sufficient. This does not
         * mean by far, that the parameter setting is valid in the sequel or
         * lifetime of this program
         */
        if (!ServoMotion.validateForImpedanceMode(lbr))
        {
            getLogger()
                    .info("Validation of torque model failed - correct your mass property settings");
            getLogger()
                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
        }
    }

    @Override
    public void run()
    {
        getLogger().info("Move to start position.");
        moveToInitialPosition();

        // Initialize Cartesian impedance control mode
        final CartesianImpedanceControlMode cartImp = createCartImp();

        getLogger()
                .info("Sample Application - SmartServoLIN motion in cartesian impedance control mode");
        runSmartServoLINMotion(cartImp);

        // Return to initial position
        moveToInitialPosition();

        // Initialize position control mode
        final PositionControlMode positionCtrlMode = new PositionControlMode();

        getLogger()
                .info("Sample Application -  SmartServoLIN motion in position control mode");
        runSmartServoLINMotion(positionCtrlMode);
    }

    /**
     * Creates a smartServoLIN Motion with the given control mode and moves
     * around.
     * 
     * @param controlMode
     *            the control mode which shall be used
     * @see {@link CartesianImpedanceControlMode}
     */

    protected void runSmartServoLINMotion(final IMotionControlMode controlMode)
    {
        AbstractFrame initialPosition = lbr.getCurrentCartesianPosition(lbr
                .getFlange());

        // Create a new smart servo linear motion
        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

        aSmartServoLINMotion.setMaxTranslationVelocity(maxTranslationVelocity);
        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServoLIN in " + controlMode);
        lbr.moveAsync(aSmartServoLINMotion.setMode(controlMode));

        getLogger().info("Get the runtime of the SmartServoLIN motion");
        theSmartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        StatisticTimer timing = new StatisticTimer();

        getLogger().info("Do sine movement");
        timing = startSineMovement(theSmartServoLINRuntime, timing, controlMode);

        ThreadUtil.milliSleep(500);

        // Print statistic timing
        getLogger().info(
                getClass().getName() + theSmartServoLINRuntime.toString());

        getLogger().info("Stop the SmartServoLIN motion");
        theSmartServoLINRuntime.stopMotion();

        // Statistic Timing of sine movement loop
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger()
                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger()
                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
        }
    }

    /**
     * Create the CartesianImpedanceControlMode class for motion
     * parameterization.
     * 
     * @see {@link CartesianImpedanceControlMode}
     * @return the created control mode
     */
    private CartesianImpedanceControlMode createCartImp()
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        cartImp.parametrize(CartDOF.Z).setStiffness(800.0);
        return cartImp;
    }

    private StatisticTimer startSineMovement(
            ISmartServoLINRuntime theSmartServoLINRuntime,
            StatisticTimer timing, IMotionControlMode mode)
    {
        Frame aFrame = theSmartServoLINRuntime
                .getCurrentCartesianDestination(lbr.getFlange());

        try
        {
            getLogger().info("Start SmartServoLIN sine movement");
            double omega = freqency * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
            int i;

            for (i = 0; i < numRuns; ++i)
            {
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system

                ThreadUtil.milliSleep(milliSleepToEmulateComputationalEffort);

                // Update the smart servo LIN runtime
                theSmartServoLINRuntime.updateWithRealtimeSystem();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // Compute the sine function
                Frame destFrame = new Frame(aFrame);
                destFrame.setZ(amplitude * Math.sin(sinArgument));

                // Set new destination
                theSmartServoLINRuntime.setDestination(destFrame);
                aStep.end();
            }

            // Modify the stiffness settings every now and then
            if (i % (numRuns / 10) == 0)
            {
                if (mode instanceof CartesianImpedanceControlMode)
                {
                    // We are in CartImp Mode,
                    // Modify the settings:
                    // NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
                    // NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
                    // WILL DESTABILIZE THE CONTROLLER
                    final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) mode;
                    final double aTransStiffVal = Math.max(100. * (i
                            / (double) numRuns + 1), 1000.);
                    final double aRotStiffVal = Math.max(10. * (i
                            / (double) numRuns + 1), 150.);
                    cartImp.parametrize(CartDOF.TRANSL).setStiffness(
                            aTransStiffVal);
                    cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);

                    // Send the new Stiffness settings down to the
                    // controller
                    theSmartServoLINRuntime.changeControlModeSettings(cartImp);
                }
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
    public static void main(final String[] args)
    {
        final SmartServoLINInteractionControl app = new SmartServoLINInteractionControl();
        app.runApplication();
    }
}
