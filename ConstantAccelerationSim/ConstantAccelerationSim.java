import org.json.JSONArray;
import org.json.JSONML;
import org.json.simple.JSONObject;
import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class ConstantAccelerationSim {

    static JSONObject point = new JSONObject();

    static JSONArray pathPointsJSON = new JSONArray();

    static double turnTimePercent = 0.8;

    static double velocityX = 0.0;
    static double velocityY = 0.0;

    static double originalX = 0.0;
    static double originalY = 0.0;

    static double currentX = 0.0;
    static double currentY = 0.0;

    static double currentTheta = 0.0;
    static double thetaChange = 0.0;

    static Boolean inCurve = false;
    static Boolean needCurve = false;

    public static void main(String[] args) {
        
        for(int i = 0; i <= 7; i++) {
            if(i == 2) {
                point.put("X", 4.0);
                point.put("Y", 1.0);
                point.put("Theta", i * 18);
                point.put("Time", (i * 10) -2);
                pathPointsJSON.put(point);
            }
            else {
                point.put("X", i * 2.0);
                point.put("Y", i * 2.0);
                point.put("Theta", i * 18);
                point.put("Time", i * 10);
                pathPointsJSON.put(point);
            }
            if(i > 5) {
                point.put("X", 10.0);
                point.put("Y", 10.0);
                point.put("Theta", i * 18);
                point.put("Time", i * 10);
                pathPointsJSON.put(point);
            }
            
        }
        System.out.println(pathPointsJSON.toString());

        org.json.JSONObject currentPoint = pathPointsJSON.getJSONObject(1);
        org.json.JSONObject hookPoint = pathPointsJSON.getJSONObject(2);
        org.json.JSONObject thirdPoint = pathPointsJSON.getJSONObject(3);

        for(int i = 0; i < 60; i++) {
            if(inCurve == false) {
                currentPoint = pathPointsJSON.getJSONObject(i/10);
                hookPoint = pathPointsJSON.getJSONObject((i/10) + 1);
                thirdPoint = pathPointsJSON.getJSONObject((i/10) + 2);
            }           
            int currentPointTime = currentPoint.getInt("Time");
            int hookPointTime = hookPoint.getInt("Time");
            int thirdPointTime = thirdPoint.getInt("Time");

            int timeDiffT1 = (hookPointTime - currentPointTime);
            double t1 = (timeDiffT1 * turnTimePercent) + currentPointTime;

            int timeDiffT2 = (thirdPointTime - hookPointTime);
            double t2 = (timeDiffT2 * (1-turnTimePercent)) + hookPointTime;

            // System.out.println("T1: " + t1);
            // System.out.println("T2: " + t2);

            // System.out.println("Current: " + currentPointTime + " Next: " + hookPointTime);

            double t1X = (hookPoint.getDouble("X") - currentPoint.getDouble("X"))/timeDiffT1;
            double t1Y = (hookPoint.getDouble("Y") - currentPoint.getDouble("Y"))/timeDiffT1;

            double t2X = (thirdPoint.getDouble("X") - hookPoint.getDouble("X"))/timeDiffT2;
            double t2Y = (thirdPoint.getDouble("Y") - hookPoint.getDouble("Y"))/timeDiffT2;

            if(t1X == t2X && t1Y == t2Y) {
                needCurve = false;
            }
            else {
                needCurve = true;
            }

            if(needCurve == false || i < t1) {
                // replace currentPoint xy with odometry xy
                velocityX = (hookPoint.getDouble("X") - currentPoint.getDouble("X"))/timeDiffT1;
                // System.out.println("velocityX " + velocityX);
                velocityY = (hookPoint.getDouble("Y") - currentPoint.getDouble("Y"))/timeDiffT1;

                thetaChange = ((hookPoint.getDouble("Theta") - currentPoint.getDouble("Theta"))/timeDiffT1);

                currentX = (velocityX * i) + originalX;
                currentY = (velocityY * i) + originalY;

                System.out.println("Time: " + i  + " VelocityX: " + velocityX + " VelocityY: " + velocityY + " AnglePerSec: " + thetaChange);
            }
            else {
                needCurve = true;
                // instead of calculating where it would be use odometry x and y
                double t1Theta = ((hookPoint.getDouble("Theta")  - currentPoint.getDouble("Theta")) * ((t1 - 1)%timeDiffT1)/timeDiffT1) + currentPoint.getDouble("Theta");
                double t2Theta = ((thirdPoint.getDouble("Theta") - hookPoint.getDouble("Theta")) * ((t2 -1)%timeDiffT2)/timeDiffT2) + hookPoint.getDouble("Theta");

                System.out.println("Time: " + i + " t1Theta: " + t1Theta + " t2Theta: " + t2Theta);

                double angleChangeRate = (t2Theta - t1Theta)/(t2 - t1);

                double accelX = (t2X - t1X)/(t2 - t1);
                double accelY = (t2Y - t1Y)/(t2 - t1);

                // System.out.println("AccelX: " + accelX);

                double adjustVelocityX = (accelX * (i - t1)) + velocityX;
                double adjustVelocityY = (accelY * (i - t1)) + velocityY;

                // if((Math.abs(adjustVelocityX - t2X) < 0.05 && Math.abs(adjustVelocityY - t2Y) < 0.05)) {
                //     System.out.println("Inside finish");
                //     // i = (int) t2;
                //     needCurve = false;
                // }

                if((Math.abs(adjustVelocityX - t2X) == 0 && Math.abs(adjustVelocityY - t2Y) == 0)) {
                    System.out.println("Inside finish");
                    // i = (int) t2;
                    needCurve = false;
                }

                System.out.println("Time: " + i  + " Hook time: " + t1 + " T2: " + t2 + " Adj Velocity X: " + adjustVelocityX + " Adj Velocity Y: " + adjustVelocityY + " AnglePerSec: " + angleChangeRate);
            }
        }
    }

}
