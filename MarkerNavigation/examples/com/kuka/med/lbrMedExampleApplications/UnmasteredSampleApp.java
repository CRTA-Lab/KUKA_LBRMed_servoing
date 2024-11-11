package com.kuka.med.lbrMedExampleApplications;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.med.mastering.Mastering;
import com.kuka.med.unmasteredApp.MedApplicationCategory;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PTPHome;
import com.kuka.task.ITaskLogger;

/**
 * This example shows how to master the axis number 2 of the robot. If the axis is already mastered, it invalidates the
 * mastering first. The application can even start if the axis is not mastered, due to the annotation
 * "@MedApplicationCategory(checkMastering = false)".
 */
@MedApplicationCategory(checkMastering = false)
public class UnmasteredSampleApp extends RoboticsAPIApplication
{
    @Inject
    private ITaskLogger _logger;

    @Inject
    private LBR _lbr;

    private static final int MILLI_SLEEP_TO_PRODUCE_PAUSE = 2000;

    @Override
    public void run()
    {
        //Create a mastering object
        Mastering mastering = new Mastering(_lbr);

        //Invalidate Axis 2 mastering if it is already mastered 
        if (mastering.isAxisMastered(1))
        {
            _logger.info("Axis 2 is mastered ... invalidating its mastering");
            mastering.invalidateMastering(1);
        }

        //Master Axis 2
        _logger.info("Mastering Axis 2...");
        if (mastering.masterAxis(1))
        {
            _logger.info("Mastering Axis 2 finished");
        }
        else
        {
            _logger.error("Mastering Axis 2 failed");
        }

        ThreadUtil.milliSleep(MILLI_SLEEP_TO_PRODUCE_PAUSE);

        //Move to home position (will fail if not all axes are mastered) 
        //WARNING: Make sure, that the pose is collision free!
        _logger.info("Moving home...");
        _lbr.move(new PTPHome().setJointVelocityRel(0.2));

        _logger.info("Application finished");
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(final String[] args)
    {
        final UnmasteredSampleApp app = new UnmasteredSampleApp();
        app.runApplication();
    }

}
