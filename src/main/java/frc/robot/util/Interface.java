package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriverConstants;


import frc.robot.util.CameraWebsocketClient.Apriltag;
import frc.robot.util.CameraWebsocketClient.Info;
import frc.robot.util.CameraWebsocketClient.Color;

public class Interface {
    private String ip;
    private ArrayList<CameraWebsocketClient> camClientList = new ArrayList<CameraWebsocketClient>();
    private HashMap<String, Integer> apriltagAngles;

    private PIDController turnPID = new PIDController(0.1, 0.0, 0.0);
    private PIDController movePID = new PIDController(0.1, 0.0, 0.0);
    
    public Interface(String ipAddress, int[] cameraRotation, HashMap<String, Integer> apriltagAngles) {
        // This constructor is not ideal but it works for the example. IRL you would want to use the other constructor so you can still have a list of cameras outside of the estimator.
        // Camera Rotation is the rotation of each camera in degrees. 0 is the default rotation.
        // Apriltag Angles is a hashmap of the apriltag id to the angle of the tag in degrees. 0 is facing the camera.

        this.ip = ipAddress;
        this.apriltagAngles = apriltagAngles;
        boolean failed = false;
        int i = 0;
        while(!failed) {
            System.out.println("Trying to commect to: " + ip + ":" + (i + 50000));
            CameraWebsocketClient newCam = new CameraWebsocketClient(ip + ":" + (i + 50000), 1000);
            newCam.setupConnection();

            try {
                // Wait for 1 second (1000 milliseconds)
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if(newCam.isConnected()) {
                System.out.println("Connected to: " + ip + ":" + (i + 50000));
                newCam.setRotation(cameraRotation[i]);
                camClientList.add(newCam);
                i++;
            } else {
                failed = true;
            }
        }

        turnPID.disableContinuousInput();
        turnPID.setSetpoint(0);
        movePID.disableContinuousInput();
        movePID.setSetpoint(0);

    }

    public Interface(CameraWebsocketClient[] camList, HashMap<String, Integer> apriltagAngles) {
        this.apriltagAngles = apriltagAngles;
        for(CameraWebsocketClient newCam : camList) {
            if(newCam.isConnected()) {
                camClientList.add(newCam);
            }
        }
        turnPID.disableContinuousInput();
        turnPID.setSetpoint(0);
        movePID.disableContinuousInput();
        movePID.setSetpoint(0);
    }

    public double getZAngle(int maxTags) {
        // This function returns the average calculated angle of the robot in degrees on the z axis, aka the only one the robot turns on. Limit the number of tags to use with maxTags if you want.

        double ZAngle = 0;
        double numTags = 0;
        for(CameraWebsocketClient cam : camClientList) {
            List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();

            System.out.println(tags.size());

            for (CameraWebsocketClient.Apriltag tag : tags) {
                int tagAngle = apriltagAngles.getOrDefault(tag.tagId, 0);
                ZAngle += tagAngle * (180/Math.PI) + cam.getRotation() + tag.orientation[1];
                numTags++;
                if(numTags == maxTags) {
                    break;
                }
            }
            if(numTags == maxTags) {
                break;
            }
        }
        if(numTags == 0) {
            return 69420.0; // nice
        }
        return ZAngle/numTags % 360;
    }

    public double getZAngle() {

        return getZAngle(4);
    }

    public ChassisSpeeds getTagDrive(int camIndex, String tagId) {
        // This function returns the relative position of the tag to the camera in the camera's frame of reference.
        // The position is returned as a 3 element array of doubles in the form [x, y, z]
        // The position is in meters.

        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        Apriltag tag = null;
        for (Apriltag t : tags) {
            if(t.tagId.equals(tagId)) {
                tag = t;
                break;
            }
        }
        if(tag == null) {
            return null;
        }

        double turnSpeed = turnPID.calculate(tag.orientation[1]);
        double moveSpeed = movePID.calculate(tag.distance);

        double xMove = (tag.position[2] / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;
        double yMove = (tag.position[0] / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;
        
        System.out.println("X: " + xMove + " Y: " + yMove);
        return new ChassisSpeeds(
            DriverConstants.highDriveSpeed * xMove,
            DriverConstants.highDriveSpeed * yMove,
            turnSpeed
        );

    }

    public Info getInfo() {
        return camClientList.get(0).getInfo();
    }

    public static void main(String[] args){
        HashMap<String, Integer> apriltagAngles = new HashMap<>();
        apriltagAngles.put("tag1", 30);
        apriltagAngles.put("tag2", 45);
        apriltagAngles.put("tag3", 60);
        apriltagAngles.put("tag4", 90);

        int[] cameraRotation = {0, 90, 180, 270};

        Interface estimator = new Interface("ws://10.42.0.118", cameraRotation, apriltagAngles);
        

        while (true) {
            estimator.getTagDrive(0, "13");
            //System.out.println(estimator.getZAngle());
        }
    }
        
}
