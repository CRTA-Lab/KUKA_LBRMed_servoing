package com.kuka.med.lbrMedExampleApplications;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.med.controllerModel.MedController;
import com.kuka.med.robotStateEvent.IRobotStateListener;
import com.kuka.med.robotStateEvent.RobotStateEvent;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a sample application handling the robot state event mechanism. The IRobotStateListener is used to
 * receive events about changes of the robot state.
 */
public class RobotStateEventSampleApp extends RoboticsAPIApplication implements IRobotStateListener
{
    private MedController _medController;

    @Inject
    private LBR _lbr;

    @Inject
    private ITaskLogger _logger;

    // History of the events
    private List<RobotStateEvent> _eventHistory = new ArrayList<RobotStateEvent>();

    private boolean _robotStateEventReceived = false;
    private boolean _continueApplication = true;

    private static final long SLEEP_TIME = 500L;

    @Override
    public void initialize()
    {
        _medController = (MedController) _lbr.getController();
        // register the application as robot state listener 
        _medController.addRobotStateListener(this, _lbr);
    }

    @Override
    public void dispose()
    {
        // remove listener registration of this application
        _medController.removeRobotStateListener(this);
        super.dispose();
    }

    @Override
    public void run()
    {
        while (_continueApplication)
        {
            if (_robotStateEventReceived)
            {
                // reset event flag
                _robotStateEventReceived = false;
                // ask user for action
                int userChoice = getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION,
                        "New robot state event(s) received",
                        "Print last event", "Print event history", "Clear event history", "Continue application", "Exit application");

                // evaluate user choice
                switch (userChoice)
                {

                // Choice: Print last received event
                case 0:
                    _logger.info("======== LAST ROBOT STATE EVENT ========");
                    printEventInfo(_eventHistory.get(_eventHistory.size() - 1));
                    break;

                // Choice: Print event history
                case 1:
                    printEventHistory();
                    break;

                // Choice: Clear event history
                case 2:
                    _eventHistory.clear();
                    break;

                // Choice: Continue application
                case 3:
                    break;

                // Choice: Exit application
                case 4:
                    _continueApplication = false;
                    break;

                // Choice: unknown
                default:
                    _logger.error("Undefined choice");
                    break;
                }
            }

            ThreadUtil.milliSleep(SLEEP_TIME);
        }
    }

    private void printEventInfo(RobotStateEvent event)
    {
        _logger.info(event.toString());
    }

    private void printEventHistory()
    {
        _logger.info("************ TRIGGER EVENT HISTORY ************");
        int idx = 0;
        for (RobotStateEvent event : _eventHistory)
        {
            _logger.info("======== ROBOT STATE EVENT " + idx++ + " ========");
            printEventInfo(event);
        }
        _logger.info("************************************");
    }

    @Override
    public void onRobotStateChanged(RobotStateEvent event)
    {
        // update event history
        _eventHistory.add(event);
        _robotStateEventReceived = true;
    }
}
