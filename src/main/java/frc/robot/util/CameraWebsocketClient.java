package frc.robot.util;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonArray;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

import javax.websocket.*;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

@ClientEndpoint
public class CameraWebsocketClient {
    private String ip = "ws://10.42.0.118:50000";
    private Session wSession;
    private double rotation;

    public static class Color {
        public double red;
        public double green;
        public double blue;
        public double difference;
        public double blur;
    }

    public static class Info {
        public String cameraName;
        public String identifier;
        public double horizontalFocalLength;
        public double verticalFocalLength;
        public int height;
        public int horizontalResolutionPixels;
        public int verticalResolutionPixels;
        public int processingScale;
        public double tiltAngleRadians;
        public double horizontalFieldOfViewRadians;
        public double verticalFieldOfViewRadians;
        public List<Color> colorList;
        public int activeColor;
        public String fullString;
    }

    public static class Apriltag {
        public String tagId;
        public double[] position;
        public double[] orientation;
        public double distance;
        public double horizontalAngle;
        public double verticalAngle;
        public String fullString;
    }

    @OnMessage
    public void onMessage(String newMessage) {
        // This is the part that is skechy - writing to a file isn't the best way to do this but it works for the example. 
        try {
            FileWriter myWriter = new FileWriter("file_" + ip.replace("/", "").replace(":", "") + ".txt");
            myWriter.write(newMessage);
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public CameraWebsocketClient() {}

    public CameraWebsocketClient(String ip) {
        // Ip should be somehting like "ws://10.42.0.118:50000". Include the ws:// and the port number.
        this.ip = ip;
    }

    @OnOpen
    public void onOpen(Session session) {
        System.out.println("Connected to WebSocket server");
    }

    @OnClose
    public void onClose(Session session, CloseReason closeReason) {
        System.out.println("Disconnected from WebSocket server");
        System.out.println("Reason: " + closeReason);
    }

    @OnError
    public void onError(Session session, Throwable throwable) {
        System.out.println("An error occurred:");
        throwable.printStackTrace();
    }

    public boolean isConnected() {
        if (wSession == null) {
            return false;
        }
        return wSession.isOpen();
    }

    public boolean setupConnection() {
        // This function sets up the connection to the websocket server. It returns true if the connection was successful and false if it was not.
        // Call this at any time if you want to reconnect to the server.
        try {
            WebSocketContainer container = ContainerProvider.getWebSocketContainer();
            URI uri = new URI(this.ip);
            wSession = container.connectToServer(CameraWebsocketClient.class, uri);
            Thread.sleep(1000);
            if (isConnected()) {
                return true;
            } else {
                return false;
            }
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
        
    }

    private void sendMessage(Session session, String pMessage) {
        session.getAsyncRemote().sendText(pMessage);
    }

    public String getMessage() {
        // This function gets the message from the websocket server. It returns the message as a string.
        // This is the other sketchy part - it reads from a file. This is not the best way to do this but it works for the example.

        double startTime = System.currentTimeMillis();
        String data = "";

        while (startTime + 5000 > System.currentTimeMillis()) {
            try {
                boolean breakLoop = false;
                File myObj = new File("file_" + ip.replace("/", "").replace(":", "") + ".txt");
                Scanner myReader = new Scanner(myObj);
                while (myReader.hasNextLine()) {
                    data = myReader.nextLine();
                    breakLoop = true;
                }
                if (breakLoop) {
                    break;
                }
                myReader.close();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
        try {
            FileWriter myWriter = new FileWriter("file_" + ip.replace("/", "").replace(":", "") + ".txt");
            myWriter.write("");
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return data;
        
    }

    public JsonObject decodeJson(String jsonString) {
        Gson gson = new Gson();
        return gson.fromJson(jsonString, JsonObject.class);
    }

    public double getRotation() {
        return rotation;
    }

    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

    public Info getInfo() {
        try {
            sendMessage(wSession, "info");
            String newMessage = getMessage();
            return getInfoFromString(newMessage);
        } catch (Exception e) {
            return new Info();
        }
    }

    private Info getInfoFromString(String pMessage) {
        try {
            JsonObject json = decodeJson(pMessage);
            Info info = new Info();
            info.cameraName = json.get("cam_name").getAsString();
            info.identifier = json.get("identifier").getAsString();
            info.horizontalFocalLength = json.get("horizontal_focal_length").getAsDouble();
            info.verticalFocalLength = json.get("vertical_focal_length").getAsDouble();
            info.height = json.get("height").getAsInt();
            info.horizontalResolutionPixels = json.get("horizontal_resolution_pixels").getAsInt();
            info.verticalResolutionPixels = json.get("vertical_resolution_pixels").getAsInt();
            info.processingScale = json.get("processing_scale").getAsInt();
            info.tiltAngleRadians = json.get("tilt_angle_radians").getAsDouble();
            info.horizontalFieldOfViewRadians = json.get("horizontal_field_of_view_radians").getAsDouble();
            info.verticalFieldOfViewRadians = json.get("vertical_field_of_view_radians").getAsDouble();
            info.activeColor = json.get("active_color").getAsInt();
            info.colorList = new ArrayList<>();
            for (var colorJson : json.getAsJsonArray("color_list")) {
                Color color = new Color();
                JsonObject colorObject = colorJson.getAsJsonObject();
                color.red = colorObject.get("red").getAsDouble();
                color.green = colorObject.get("green").getAsDouble();
                color.blue = colorObject.get("blue").getAsDouble();
                color.difference = colorObject.get("difference").getAsDouble();
                color.blur = colorObject.get("blur").getAsDouble();
                info.colorList.add(color);
            }
            info.fullString = pMessage;
            return info;
        } catch (Exception e) {
            System.out.println("Error getting info");
            e.printStackTrace();
            return new Info();
        }
    }

    public List<Apriltag> getApriltags() {
        try {
            sendMessage(wSession, "fa");
            String newMessage = getMessage();
            return getApriltagsFromString(newMessage);
        } catch (Exception e) {
            e.printStackTrace();
            if (setupConnection()){
                sendMessage(wSession, "fa");
                String newMessage = getMessage();
                return getApriltagsFromString(newMessage);
            }
            return new ArrayList<Apriltag>();
        }
    }


    private List<Apriltag> getApriltagsFromString(String pMessage) {
        try {
            // Decode the JSON string into a JsonArray
            JsonArray jsonArray = new Gson().fromJson(pMessage, JsonArray.class);
            List<Apriltag> apriltags = new ArrayList<>();
    
            // Check if the JsonArray is not null and has elements
            if (jsonArray != null && jsonArray.size() > 0) {
                for (var apriltagJson : jsonArray) {
                    Apriltag apriltag = new Apriltag();
                    JsonObject apriltagObject = apriltagJson.getAsJsonObject();
                    apriltag.tagId = apriltagObject.get("tag_id").getAsString();
                    apriltag.position = new double[]{
                        apriltagObject.get("position").getAsJsonArray().get(0).getAsDouble(),
                        apriltagObject.get("position").getAsJsonArray().get(1).getAsDouble(),
                        apriltagObject.get("position").getAsJsonArray().get(2).getAsDouble()
                    };
                    apriltag.orientation = new double[]{
                        apriltagObject.get("orientation").getAsJsonArray().get(0).getAsDouble(),
                        apriltagObject.get("orientation").getAsJsonArray().get(1).getAsDouble(),
                        apriltagObject.get("orientation").getAsJsonArray().get(2).getAsDouble()
                    };
                    apriltag.distance = apriltagObject.get("distance").getAsDouble();
                    apriltag.horizontalAngle = apriltagObject.get("horizontal_angle").getAsDouble();
                    apriltag.verticalAngle = apriltagObject.get("vertical_angle").getAsDouble();
                    apriltags.add(apriltag);
                }
                apriltags.get(0).fullString = pMessage;
            }
    
            return apriltags;
        } catch (Exception e) {
            e.printStackTrace();
            return new ArrayList<>();
        }
    }

    // There are a lot more functions I can write but I think this is enough for the meeting. I can write more if you want.
}