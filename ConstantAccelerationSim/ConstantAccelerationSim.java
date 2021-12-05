import org.json.JSONArray;
import org.json.JSONML;
import org.json.simple.JSONObject;

import java.lang.ref.Cleaner;
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

    static double currentVelocityX = 0;
    static double currentVelocityY = 0;
    static double currentThetaChange = 0;

    static double currentTheta = 0.0;
    static double thetaChange = 0.0;

    static double t1Theta = 0;

    static double timingResolution = 50;

    static double[] velocityArray = new double[3];

    public static double[] constantAccelerationInterpolation(double currentX, double currentXVelocity, double currentY, double currentYVelocity, double currentTheta, double currentThetaVelocity, double time, JSONArray pathPointsJSON) {

        org.json.JSONObject currentPoint;
        org.json.JSONObject nextPoint;
        org.json.JSONObject previousPoint;

        double[] returnArray = new double[3];
        int currentPointIndex = 0;
        double lowestTimeDiff = 0;

        double[] timeDiffArray = new double[pathPointsJSON.length()];

        for(int i = 0; i < pathPointsJSON.length(); i++) {
            timeDiffArray[i] = Math.abs(time - pathPointsJSON.getJSONObject(i).getDouble("Time"));
        }
        for(int i = 0; i < timeDiffArray.length; i++) {
            if(i == 0) {
                lowestTimeDiff = timeDiffArray[0];
                currentPointIndex = 0;
            }
            else {
                if(timeDiffArray[i] < lowestTimeDiff) {
                    lowestTimeDiff = timeDiffArray[i];
                    currentPointIndex = i;
                }
            }
        }

        currentPoint = pathPointsJSON.getJSONObject(currentPointIndex);

        double currentPointTime = currentPoint.getDouble("Time");

        double velocityX = 0;
        double velocityY = 0;
        double thetaChange = 0;

        if(currentPointIndex + 1 >= pathPointsJSON.length()) {
            currentPoint = pathPointsJSON.getJSONObject(currentPointIndex);
            previousPoint = pathPointsJSON.getJSONObject(currentPointIndex - 1);
            velocityX = (currentPoint.getDouble("X") - currentX)/(currentPointTime - time);
            velocityY = (currentPoint.getDouble("Y") - currentY)/(currentPointTime - time);
            thetaChange = (currentPoint.getDouble("Theta") - currentTheta)/(currentPointTime - time);
            returnArray[0] = velocityX;
            returnArray[1] = velocityY;
            returnArray[2] = thetaChange;

            return returnArray;
        }
        else if(currentPointIndex - 1 < 0) {
            currentPoint = pathPointsJSON.getJSONObject(currentPointIndex);
            nextPoint = pathPointsJSON.getJSONObject(currentPointIndex + 1);
            double nextPointTime = nextPoint.getDouble("Time");
            velocityX = (nextPoint.getDouble("X") - currentX)/(nextPointTime - time);
            velocityY = (nextPoint.getDouble("Y") - currentY)/(nextPointTime - time);
            thetaChange = (nextPoint.getDouble("Theta") - currentTheta)/(nextPointTime - time);
            returnArray[0] = velocityX;
            returnArray[1] = velocityY;
            returnArray[2] = thetaChange;

            return returnArray;
        }

        nextPoint = pathPointsJSON.getJSONObject(currentPointIndex + 1);
        previousPoint = pathPointsJSON.getJSONObject(currentPointIndex - 1);

        double nextPointTime = nextPoint.getDouble("Time");
        double previousPointTime = previousPoint.getDouble("Time");

        double timeDiffT1 = (currentPointTime - previousPointTime);
        // double timeTonextPoint = (nextPointTime - time);
        double t1 = (timeDiffT1 * turnTimePercent) + previousPointTime;

        double timeDiffT2 = nextPointTime - currentPointTime;
        double t2 = (timeDiffT2 * (1 - turnTimePercent)) + currentPointTime;

        if(time <= t1) {
            velocityX = (currentPoint.getDouble("X") - currentX)/(currentPointTime - time);
            velocityY = (currentPoint.getDouble("Y") - currentY)/(currentPointTime - time);
            thetaChange = (currentPoint.getDouble("Theta") - currentTheta)/(currentPointTime - time);
        }
        else if(time > t1 && time < t2) {
            double t1X = currentXVelocity;
            double t1Y = currentYVelocity;
            double t1Theta = currentThetaVelocity;

            double t2X = (nextPoint.getDouble("X") - currentPoint.getDouble("X"))/timeDiffT2;
            double t2Y = (nextPoint.getDouble("Y") - currentPoint.getDouble("Y"))/timeDiffT2;
            double t2Theta = (nextPoint.getDouble("Theta") - currentPoint.getDouble("Theta"))/timeDiffT2;

            double previousVelocityX = t1X;
            double previousVelocityY = t1Y;
            double previousTheta = t1Theta;

            double accelX = (t2X - t1X)/(t2 - time);
            double accelY = (t2Y - t1Y)/(t2 - time);
            double accelTheta = (t2Theta - t1Theta)/(t2 - time);

            velocityX = (accelX * (1/timingResolution)) + previousVelocityX;
            velocityY = (accelY * (1/timingResolution)) + previousVelocityY;
            thetaChange = (accelTheta * (time - t1)) + previousTheta;
        }
        else if(time >= t2) {
            velocityX = (nextPoint.getDouble("X") - currentX)/(nextPointTime - time);
            velocityY = (nextPoint.getDouble("Y") - currentY)/(nextPointTime - time);
            thetaChange = (nextPoint.getDouble("Theta") - currentTheta)/(nextPointTime - time);
        }

        returnArray[0] = velocityX;
        returnArray[1] = velocityY;
        returnArray[2] = thetaChange;

        return returnArray;
    }

    public static void main(String[] args) {
        point.put("X", 0.0);
        point.put("Y", 0.0);
        point.put("Theta", 0);
        point.put("Time", 0);
        pathPointsJSON.put(point);
        point.put("X", 0.0);
        point.put("Y", 0.0);
        point.put("Theta", 0);
        point.put("Time", 5);
        pathPointsJSON.put(point);
        point.put("X", 5.0);
        point.put("Y", 0.0);
        point.put("Theta", 0);
        point.put("Time", 10);
        pathPointsJSON.put(point);
        point.put("X", 5.0);
        point.put("Y", 5.0);
        point.put("Theta", 0);
        point.put("Time", 15);
        pathPointsJSON.put(point);

        java.util.Random r = new java.util.Random();
        // currentY = 1;
        for(int i = 0; i < (pathPointsJSON.getJSONObject(pathPointsJSON.length() - 1).getDouble("Time")) * timingResolution; i++) {
            double noiseX = r.nextGaussian() * Math.sqrt(0.0005);
            double noiseY = r.nextGaussian() * Math.sqrt(0.0005);
            double noiseTheta = r.nextGaussian() * Math.sqrt(0.0005);

            noiseX = 0;
            noiseY = 0;
            noiseTheta = 0;

            velocityArray = constantAccelerationInterpolation(currentX, currentVelocityX, currentY, currentVelocityY, currentTheta, currentThetaChange, (i/timingResolution), pathPointsJSON);

            currentVelocityX = velocityArray[0];
            currentVelocityY = velocityArray[1];
            currentThetaChange = velocityArray[2];

            currentX = noiseX + currentX + ((1/timingResolution) * currentVelocityX);
            currentY = noiseY + currentY + ((1/timingResolution) * currentVelocityY);
            currentTheta = noiseTheta + currentTheta + ((1/timingResolution) * currentThetaChange);

            System.out.println(i/timingResolution + ", " + currentX + ", " + currentY + ", " + currentTheta + ", " + currentVelocityX + ", " + currentVelocityY + ", " + currentThetaChange);

        }
    }

}
