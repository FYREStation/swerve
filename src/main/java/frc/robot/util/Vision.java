package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriverConstants;


import frc.robot.util.CameraWebsocketClient.Apriltag;
import frc.robot.util.CameraWebsocketClient.Info;


public class Vision {
    private String ip;
    private ArrayList<CameraWebsocketClient> camClientList = new ArrayList<CameraWebsocketClient>();
    private HashMap<String, Integer> apriltagAngles;

    private PIDController turnPID = new PIDController(0.1, 0.0, 0.0);
    private PIDController movePID = new PIDController(0.1, 0.0, 0.0);
    
    public Vision(String ipAddress, int[] cameraRotation, HashMap<String, Integer> apriltagAngles) {
        // This constructor is not ideal but it works for the example. IRL you would want to use the other constructor so you can still have a list of cameras outside of the Interface.
        // Maybe I will make this the only class that you need to use with the cameras then it will be fine.
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

            if(newCam.isConnected()) {
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

    public Vision(CameraWebsocketClient[] camList, HashMap<String, Integer> apriltagAngles) {
        this.apriltagAngles = apriltagAngles;
        for(CameraWebsocketClient newCam : camList) {
            if(newCam.isConnected()) {
                camClientList.add(newCam);
            }
        }

        // Erm, what the sigma? IS this corect? I think it is but I am not sure
        turnPID.disableContinuousInput();
        turnPID.setSetpoint(0);
        movePID.disableContinuousInput();
        movePID.setSetpoint(0);
    }

    public void clear(){
        for(CameraWebsocketClient cam : camClientList) {
            cam.clear();
        }
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
        // W overload
        return getZAngle(4);
    }

    public ChassisSpeeds getTagDrive(int camIndex, String tagId) {
        // This function returns the relative position of the tag to the camera in the camera's frame of reference.
        // The position is returned as a 3 element array of doubles in the form [x, y, z]
        // The position is in meters.

        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        Apriltag tag = null;
        for (Apriltag t : tags) { // This is a weird way to do this but it works - I need to make this more efficient
            if(t.tagId.equals(tagId)) {
                tag = t;
                break;
            }
        }
        if(tag == null) {
            return null;
        }

        double turnSpeed = turnPID.calculate(tag.orientation[1]); // This seems to be fine it may need to be negative but idk
        double moveSpeed = movePID.calculate(tag.distance); // I do not know if this is correct - it makes some sense but idk

        // Look at this! Max is doing a weird normalization thing again!
        double xMove = (tag.position[2] / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;
        double yMove = (tag.position[0] / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;
        
        return new ChassisSpeeds(
            DriverConstants.highDriveSpeed * xMove,
            DriverConstants.highDriveSpeed * yMove,
            turnSpeed
        );

    }

    public ChassisSpeeds getTagDrive(int camIndex) {
        // This function returns the relative position of the tag to the camera in the camera's frame of reference.
        // The position is returned as a 3 element array of doubles in the form [x, y, z]
        // The position is in meters.

        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        Apriltag tag = null;
        if (tags.size() > 0) {
            tag = tags.get(0);
        }
        if(tag == null) {
            return null;
        }

        double turnSpeed = turnPID.calculate(tag.orientation[1]); // This seems to be fine it may need to be negative but idk
        double moveSpeed = movePID.calculate(tag.distance); // I do not know if this is correct - it makes some sense but idk

        // Look at this! Max is doing a weird normalization thing again!
        double xMove = (tag.position[2] / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;
        double yMove = (tag.position[0] / (Math.abs(tag.position[0]) + Math.abs(tag.position[2]))) * moveSpeed;
        
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
        // probably cant use this main function on the bot but I used it to test
        HashMap<String, Integer> apriltagAngles = new HashMap<>();
        apriltagAngles.put("tag1", 30);
        apriltagAngles.put("tag2", 45);
        apriltagAngles.put("tag3", 60);
        apriltagAngles.put("tag4", 90);

        int[] cameraRotation = {0, 90, 180, 270};

        Vision robotInterface = new Vision("ws://10.42.0.118", cameraRotation, apriltagAngles);

        while (true) {
            robotInterface.getTagDrive(0, "13");
            //System.out.println(Interface.getZAngle());
        }
    }
        
}
