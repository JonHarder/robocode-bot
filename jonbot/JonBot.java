package jonbot;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import robocode.*;
import robocode.util.Utils;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * OneHourChallengeBot - a robot by (your name here)
 */
public class JonBot extends AdvancedRobot
{
    private static final double WALL_BUFFER = 60;
    private static final double WALL_ADJUST_RADS = Math.PI/4;

    // Compass directions (up, down, left, right) in radians
    private static final double UP = 0;
    private static final double UP_FULL = 2 * Math.PI;
    private static final double RIGHT = Math.PI/2;
    private static final double DOWN = Math.PI;
    private static final double LEFT = 1.5 * Math.PI;

    private static ArrayList<Datum> patternData = new ArrayList<>();

    private double firePower = 2.0;

    class Datum {
        double velocity;
        double headingDelta;
        double heading;
        double distance;

        Datum(double velocity, double heading, double headingDelta, double distance) {
            // these are actually used for the pattern match
            this.velocity = velocity;
            this.headingDelta = headingDelta;

            // this is only used for determining headingDelta of the next datum
            this.heading = heading;
            // this is use for determining how many ticks to simulate
            this.distance = distance;
        }
    }

    /**
     * run: JonBot's default behavior
     */
    public void run() {
        this.setAdjustGunForRobotTurn(true);
        this.setAdjustRadarForGunTurn(true);
	    // Initialization of the robot should be put here

	    // After trying out your robot, try uncommenting the import at the top,
	    // and the next line:

	    this.setColors(Color.yellow, Color.yellow, Color.yellow); // body,gun,radar

	    // Robot main loop
	    while(true) {
            this.turnRadarRight(Double.POSITIVE_INFINITY);
	    }
    }

    private double getAdjustedBearingToEnemy(@NotNull ScannedRobotEvent e, double objectHeading) {
        double enemyBearingToMyLocation = e.getBearingRadians() + this.getHeadingRadians();
        double radarDelta = enemyBearingToMyLocation - objectHeading;
        return Utils.normalRelativeAngle(radarDelta) * 2;
    }


    private double adjustHeadingAvoidCollisions(double proposedHeading) {
        // too close to top wall
        double heading = this.getHeadingRadians();
        if(this.getBattleFieldHeight() - this.getY() < WALL_BUFFER) {
            if(heading > UP && heading < RIGHT) {
                // if facing diagonally to the up/right
                return proposedHeading + WALL_ADJUST_RADS;
            } else if(heading > LEFT && heading < UP_FULL) {
                // if facing diagonally to the up/left
                return proposedHeading - WALL_ADJUST_RADS;
            }
        }

        // too close to left wall
        if(this.getX() < WALL_BUFFER) {
            if(heading > LEFT && heading < UP_FULL) {
                // if facing diagonally left/up
                return proposedHeading + WALL_ADJUST_RADS;
            } else if(heading > DOWN && heading < LEFT) {
                return proposedHeading - WALL_ADJUST_RADS;
            }
        }

        // too close to bottom wall
        if(this.getY() - this.getBattleFieldHeight() < WALL_ADJUST_RADS) {
            if(heading < DOWN && heading > RIGHT) {
                // if facing diagonally left/up
                return proposedHeading - WALL_ADJUST_RADS;
            } else if(heading > DOWN && heading < LEFT) {
                return proposedHeading + WALL_ADJUST_RADS;
            }
        }

        // too close to right wall
        if(this.getX() - this.getBattleFieldWidth() < WALL_ADJUST_RADS) {
            if(heading > UP && heading < RIGHT) {
                // if facing diagonally left/up
                return proposedHeading - WALL_ADJUST_RADS;
            } else if(heading > RIGHT && heading < DOWN) {
                return proposedHeading + WALL_ADJUST_RADS;
            }
        }

        return proposedHeading;
    }

    public void onHitRobot(HitRobotEvent e) {
    }


    /**
     * Takes the most recent set of 7 datum of enemy behavior
     * and finds the closest set in history, returning the
     * index in history of the start of the pattern.
     * @param offset The start of history to compare against
     * @return int
     */
    private int patternMatch(int offset) {
        double minDelta = Double.POSITIVE_INFINITY;
        int minDeltaIndex = 0;

        int i, j;
        Datum[] pattern = new Datum[offset];
        // initialize pattern with the most recent bot behavior of the last OFFSET ticks
        for(i=0; i<offset; i++) {
            pattern[i] = JonBot.patternData.get(i);
        }

        // loop over ever past event to see the similarity to the pattern
        for(i=offset; i<JonBot.patternData.size()-offset; i++) {
            double delta = 0;
            // compare each event of the pattern against the sample data starting at i
            for(j=0; j<offset; j++) {
                Datum patternDatum = pattern[j];
                Datum sampleDatum = JonBot.patternData.get(i+j);
                double headingDeltaDiff = Math.abs(patternDatum.headingDelta - sampleDatum.headingDelta);
                double velocityDiff = Math.abs(patternDatum.velocity - sampleDatum.velocity);
                delta += headingDeltaDiff + velocityDiff;
            }
            if(delta < minDelta) {
                minDelta = delta;
                minDeltaIndex = i;
            }
        }

        System.out.println("found closest pattern with delta of " + minDelta + " at index: " + minDeltaIndex);
        return minDeltaIndex;
    }


    /**
     * Uses the pattern of historic behavior of the enemy to replay that
     * behavior over the current situation, returning the absolute heading
     * to predicted enemy location such that firing right now with current
     * fire power, the bullet would intersect their future position.
     * @param patternStartIndex int
     * @return double
     */
    private double simulatePosition(int patternStartIndex) {
        // get the 7 data starting at patternStartIndex
        // Datum[] pattern = JonBot.patternData.

        return 3.0;
    }


    private void recordData(@NotNull ScannedRobotEvent e) {
        double heading = e.getHeading();
        double headingDelta;
        if(JonBot.patternData.size() > 0) {
            headingDelta = heading - JonBot.patternData.get(0).heading;
        } else {
            headingDelta = 0;
        }
        Datum d = new Datum(e.getVelocity(), heading, headingDelta, e.getDistance());
        JonBot.patternData.add(0, d);
        int num;
        if((num = JonBot.patternData.size()) > 500) {
            JonBot.patternData.remove(num-1);
        }
    }


    /**
     * onScannedRobot: What to do when you see another robot
     */
    public void onScannedRobot(ScannedRobotEvent e) {
        this.recordData(e);

        // if we've recorded enough data to pattern match
        if(JonBot.patternData.size() == 500) {
            Datum currentEnemyData = JonBot.patternData.get(0);
            double bulletSpeed = Rules.getBulletSpeed(this.firePower);
            int ticksToSimulate = (int)(currentEnemyData.distance/bulletSpeed);

            int patternIndex = this.patternMatch(ticksToSimulate);
            double simulatedAbsoluteHeading = this.simulatePosition(patternIndex);
        }

	    // Replace the next line with any behavior you would like
        double myAbsoluteBearing = this.getHeadingRadians() + e.getBearingRadians();

        if(e.getDistance() > 50) {
            this.setTurnGunRightRadians(Utils.normalRelativeAngle(myAbsoluteBearing - getGunHeadingRadians() +
                    (e.getVelocity() * Math.sin(e.getHeadingRadians() - myAbsoluteBearing) / 13.0)));
        } else {
            this.setTurnGunRightRadians(this.getAdjustedBearingToEnemy(e, this.getGunHeadingRadians()));
        }


        this.setTurnRadarRightRadians(this.getAdjustedBearingToEnemy(e, this.getRadarHeadingRadians()));

        double updatedPlayerHeading = this.getAdjustedBearingToEnemy(e, this.getHeadingRadians());
        double collisionAvoidedHeading = this.adjustHeadingAvoidCollisions(updatedPlayerHeading);
        this.setTurnRightRadians(collisionAvoidedHeading);

        this.firePower = this.calculateFirePower(e.getDistance());
        fire(this.firePower);

        setAhead(30);
    }


    private double calculateFirePower(double distanceToEnemy) {
        double maxDistance = Math.sqrt(Math.pow(this.getBattleFieldHeight(), 2) + Math.pow(this.getBattleFieldWidth(), 2));
        return Rules.MAX_BULLET_POWER - clamp(0, maxDistance, 0.1, 3.0, distanceToEnemy);
    }

    private static double clamp(double inputMin, double inputMax, double outputMin, double outputMax, double value) {
        return (((outputMax - outputMin) * (value - inputMin)) / (inputMax - inputMin)) + outputMin;
    }


    /**
     * onHitByBullet: What to do when you're hit by a bullet
     */
    public void onHitByBullet(HitByBulletEvent e) {
	    // Replace the next line with any behavior you would like
    }
	
    /**
     * onHitWall: What to do when you hit a wall
     */
    public void onHitWall(HitWallEvent e) {
	    // Replace the next line with any behavior you would like
    }
}
