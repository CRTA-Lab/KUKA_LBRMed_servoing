package com.navigation;


import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.IApplicationControl;
import com.kuka.roboticsAPI.applicationModel.IApplicationOverrideControl;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.*;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.RedundancyParam;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import javax.inject.Inject;
import java.lang.*;

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
public class ServoTrackingTest extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	
	private Tool pointer;
	private ObjectFrame marker;
	private Frame curr_pos;
	private Frame calib_frame;
	
	private BufferedReader in;
	private PrintWriter out;
	private ServerSocket server;
	private Socket client;
	private String msg;
	
	private ObjectFrame P1;
	private ObjectFrame P2;
	private ObjectFrame P3;
	private ObjectFrame P4;
	private ObjectFrame P5;

	@Override
	public void initialize(){
		pointer = getApplicationData().createFromTemplate("pointer_w_marker"); // creation of the tool from the template
		pointer.attachTo(robot.getFlange()); // attaching the tool to the robot flange
		marker = getApplicationData().getFrame("/fixed_marker"); // loading the fixed marker frame
		calib_frame = marker.getChild("fixed_marker_camera").copy();
		
		
		P1 = getApplicationData().getFrame("/P1");
		P2 = getApplicationData().getFrame("/P2");
		P3 = getApplicationData().getFrame("/P3");
		P4 = getApplicationData().getFrame("/P4");
		P5 = marker.getChild("P5");
		
		
		
		
		try {
			// Setup of the server socket for the communication with the camera program
			
			server = new ServerSocket(30000);
			client = server.accept();
			
			
			in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		    out = new PrintWriter(client.getOutputStream(), true);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
		
	public void Calibration(ObjectFrame[] points) throws IOException{
		msg = in.readLine();
		getLogger().info(msg);
		pointer.getFrame("TCP").move(ptp(P5).setJointVelocityRel(0.2));
		for (ObjectFrame i : points){
			pointer.getFrame("TCP").move(ptp(i).setJointVelocityRel(0.2));
			while (! msg.contains("Done")){
				//getLogger().info("Ok");
				out.println(pointer.getFrame("TCP_measurement").toStringInWorld());
				msg = in.readLine();
				//getLogger().info(msg);
			}
			msg = "OK";
		}
		out.println("Send data");
		msg = in.readLine();
		Transformation transform = parseMsg(msg);
		calib_frame.setParent(getApplicationData().getFrame("/zero_frame"));
		calib_frame.transform(transform);
		getLogger().info(calib_frame.transformationFromWorld().toString());
		return;
	}
		
	public Transformation parseMsg(String msg){
		String[] msgSplit = msg.split(",");
	    double[] data = new double[msgSplit.length-1];
	    if (msgSplit.length == 1) {
	    	//getLogger().info("No tracking data");
	    	//out.println("No data recived!");
	    	return null;
	    }
	      
	    for (int i = 1; i < msgSplit.length;i++) {
	      data[i-1] = Double.parseDouble(msgSplit[i]);
	    }
	    Vector trans = Vector.of(data[0], data[1], data[2]);
	    Rotation rot = Rotation.ofRad(data[3], data[4], data[5]);
	    Transformation fwdTransform = Transformation.of(trans, rot);
	    return fwdTransform;
	}
		
	
	public void run() {
		
		Boolean flag = true;
		int calib = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Which frame do you want to use?", "Hand teached frame", "Read frame from file","Calibrate frame with camera");
		if (calib == 2){
			try {
				out.println("Calibrate");
				
				Calibration(new ObjectFrame[]{P2,P3,P4,P1});
			} catch (IOException e) {
				e.printStackTrace();
				return;
			}
		}else{
			if (calib == 1){
				out.println("Load");
				try {
					msg = in.readLine();
					Transformation transform = parseMsg(msg);
					calib_frame.setParent(getApplicationData().getFrame("/zero_frame"));
					calib_frame.transform(transform);
				} catch (IOException e) {
					e.printStackTrace();
				}
			}else{
				out.println("Navigate");
				flag = false;
				}
		}
	    
	}
	
	@Override
	public void dispose(){
		try {
			server.close();
		
			client.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
}