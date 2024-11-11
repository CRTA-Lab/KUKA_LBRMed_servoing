package com.kuka.med.lbrMedExampleApplications;

import java.util.EnumSet;
import java.util.Set;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.med.jogging.Jogging;
import com.kuka.med.jogging.JoggingStopCondition;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.CartesianCoordinates;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.CartesianJoggingMode;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.persistenceModel.PersistenceException;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a sample application to jog from the application instead of using the keys on the smartPAD.
 * Examples for jogging on multiple axes/Cartesian coordinates with runtime update, time-specific jogging command and
 * enable jogging stop motion condition are provided.
 * 
 * NOTE: A new frame with name "BASE_FRAME" attached to the World frame must be defined in the Sunrise Project, before
 * running this application. Please, right click on the World frame in the Application Data view and select
 * "Insert new empty frame".
 * 
 */
public class JoggingSampleApp extends RoboticsAPIApplication
{

    private static final double JOGGING_VEL = 0.3;
    private static final long JOGGING_TIME = 3000;
    private static final long TIMEOUT_DISABLED = -1;
    private static final String FRAME_NAME = "/BASE_FRAME";

    private static final JointPosition HOME = new JointPosition(
            Math.toRadians(4),
            Math.toRadians(-45),
            Math.toRadians(-2),
            Math.toRadians(100),
            Math.toRadians(2),
            Math.toRadians(-34),
            Math.toRadians(16));

    @Inject
    private LBR _lbr;
    private ITaskLogger _logger;

    @Override
    public void initialize()
    {
        _logger = getLogger();
    }

    @Override
    public void run()
    {
        // Move to initial position to start jogging from the application
        _lbr.move(new PTP(HOME).setJointVelocityRel(JOGGING_VEL));

        Jogging jogging = new Jogging(_lbr);

        /* 1. Jogging on single or multiple axes with runtime override update and disabled timeout condition. Note: the
         * runtime update of the jogging parameters overrides the previous ones; thus if the jogging motion is desired
         * on one/set of axes/coordinates which are already active, they must be specified again in the new jogging
         * parameters (e.g. jogging on joint J2). */
        _logger.info("Joint jogging on axis J2.");
        jogging.startJointJogging(EnumSet.of(JointEnum.J2), JOGGING_VEL, TIMEOUT_DISABLED);
        ThreadUtil.milliSleep(JOGGING_TIME); //[ms] 
        jogging.updateJointJogging(EnumSet.range(JointEnum.J2, JointEnum.J4), JOGGING_VEL);
        _logger.info("Runtime update: joint jogging on axes J2-J4.");
        ThreadUtil.milliSleep(JOGGING_TIME); //[ms]
        jogging.stopJointJogging(EnumSet.range(JointEnum.J1, JointEnum.J7));
        _logger.info("Stop joint jogging.");

        /* 2. Time-specific joint jogging on J2 axis with disabled stop motion condition. */
        jogging.stopJoggingAtAxisLimitOrSingularity(false, true);
        jogging.startJointJogging(EnumSet.of(JointEnum.J2), -JOGGING_VEL, JOGGING_TIME);
        _logger.info("Time-specific jogging on axes J2");
        if (waitUntilJoggingIsFinished(jogging, 100, JOGGING_TIME))
        {
            _logger.info("Jogging timeout expired after " + JOGGING_TIME + " [ms]");
        }

        /* 3. Cartesian jogging on external Base Frame with enabled stop motion condition. */
        try
        {
            jogging.setBase(getApplicationData().getFrame(FRAME_NAME));
        }
        catch (PersistenceException exc)
        {
            _logger.error("Frame " + FRAME_NAME + " is not found. Using default robot base frame instead.", exc);
        }
        Set<CartesianCoordinates> coordinates = EnumSet.of(CartesianCoordinates.Z);
        jogging.startCartesianJogging(coordinates, JOGGING_VEL, CartesianJoggingMode.BASE);
        _logger.info("Cartesian jogging in BASE frame along coordinates " + coordinates.toString());
        if (!waitUntilJoggingIsFinished(jogging, 100, JOGGING_TIME))
        {
            jogging.stopCartesianJogging(coordinates);
        }
        if (jogging.getFiredStopCondition().equals(JoggingStopCondition.MOTION_IMPOSSIBLE))
        {
            _logger.info("Axis limit or singularity occurred");
        }
    }

    private boolean waitUntilJoggingIsFinished(Jogging jogging, long timeStamp, long timeout)
    {
        int count = 0;
        while (jogging.isJoggingActive())
        {
            if (count > timeout)
            {
                return false;
            }
            ThreadUtil.milliSleep(timeStamp);
            count += timeStamp;
        }
        return true;
    }

}
