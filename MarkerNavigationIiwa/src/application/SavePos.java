package application;


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class SavePos extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	
	private BufferedReader in;
	private PrintWriter out;
	private Socket client;
	private String msg;
	
	private Frame ref_frame;


	@Override
	public void initialize() {
		
		ObjectFrame marker = getApplicationData().getFrame("/zero_frame");
		ref_frame = marker.copy();
		try {
			client = new Socket("192.168.0.167",6666);

			in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		    out = new PrintWriter(client.getOutputStream(), true);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		getLogger().info("Init done");
	}

	@Override
	public void run() {
		int id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION, 
				"Record data", 
				"Exit", "Record"
		);
		if(id == 1){
			
		msg = String.valueOf(ref_frame.transformationTo(robot.getFlange()).getX());
		msg = msg + ',' + String.valueOf(ref_frame.transformationTo(robot.getFlange()).getY());
		msg = msg + ',' + String.valueOf(ref_frame.transformationTo(robot.getFlange()).getZ());
		msg = msg + ',' + String.valueOf(ref_frame.transformationTo(robot.getFlange()).getAlphaRad());
		msg = msg + ',' + String.valueOf(ref_frame.transformationTo(robot.getFlange()).getBetaRad());
		msg = msg + ',' + String.valueOf(ref_frame.transformationTo(robot.getFlange()).getGammaRad());
		out.println(msg);
		}
		out.println("Done");
		return;
	}
	public void dispose(){
		try {
		
			client.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}