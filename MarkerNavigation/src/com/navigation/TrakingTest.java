package com.navigation;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.*;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.roboticsAPI.motionModel.RedundancyParam;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;

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
public class TrakingTest extends RoboticsAPIApplication {
	@Inject
	private LBRMed robot;
	
	private Tool pointer;
	private ObjectFrame marker;
	
	private BufferedReader in;
	private PrintWriter out;
	private ServerSocket server;
	private Socket client;
	private String msg;

	@Override
	public void initialize(){
		pointer = getApplicationData().createFromTemplate("pointer");
		marker = getApplicationData().getFrame("/fixed_marker");
		pointer.attachTo(robot.getFlange());
		try {
			server = new ServerSocket(30000);
			client = server.accept();
			
			
			in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		    out = new PrintWriter(client.getOutputStream(), true);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
		

	public void run() {
		while (true){
		try {
			msg = in.readLine();
		} catch (IOException e) {
			e.printStackTrace();
			break;
		}
	    
	    getLogger().info(msg);
	    
	    if (msg == null || msg == "0") {
		      getLogger().info("No tracking data");
		      out.println("No data recived!");
		      continue;
		    }
	    
	    String[] msgSplit = msg.split(",");
	    double[] data = new double[msgSplit.length-1];
	    if (msgSplit.length == 1) {
	      getLogger().info("No tracking data");
	      out.println("No data recived!");
	      continue;
	    }
	      
	    for (int i = 1; i < msgSplit.length;i++) {
	      data[i-1] = Double.parseDouble(msgSplit[i]);
	    }
	    Frame target = marker.getChild("target").copy();
	    Frame target2 = marker.getChild("target_2").copyWithRedundancy();
	    Vector trans = Vector.of(data[0], data[1], data[2]);
	    Double datap = data[3];
	    getLogger().info(datap.toString());
	    Rotation rot = Rotation.ofRad(data[3], data[4], data[5]);
	    getLogger().info(rot.toString());
	    Transformation fwdTransform = Transformation.of(trans, rot);
	    Transformation bcwTransform = fwdTransform.invert();
	    target.transform(fwdTransform);
	    target2.setParent(target);
	    target2.transform(Transformation.ofTranslation(0, 0, 50));
	    
	    //getLogger().info(target.getRedundancyInformationForDevice(robot).toString());
	    //IRedundancyCollection redundancy = target.getRedundancyInformationForDevice(robot);
	    //getLogger().info(target.toString());
	    //pointer.getFrame("pointer_TCP").move(ptp(target2).setJointVelocityRel(0.3));
	    pointer.getFrame("pointer_TCP").move(ptp(target2).setJointVelocityRel(0.2));
	    
	    out.println("done!");
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