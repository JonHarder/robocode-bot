package jonbot;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * JonBot - a robot by Jon
 */
public class JonBot extends AdvancedRobot
{
    private static final double WALL_BUFFER = 60;
    private static final double WALL_ADJUST_RADS = Math.PI/4;
    private static final int MAX_HISTORY_ITEMS = 20000;
    private static final int MIN_HISTORY_ITEMS = 500;

    // Compass directions (up, down, left, right) in radians
    private static final double UP = 0;
    private static final double UP_FULL = 2 * Math.PI;
    private static final double RIGHT = Math.PI/2;
    private static final double DOWN = Math.PI;
    private static final double LEFT = 1.5 * Math.PI;

    private static ArrayList<Datum> patternData = new ArrayList<>();

    private double firePower = 2.0;
    private Point2D.Double lastPredictedLocation;
    private Point2D.Double[] predictedPath;

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
        this.lastPredictedLocation = null;
        this.predictedPath = null;
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
     * @param offset The start of history to compare against, also the number of events to search for a pattern
     * @return int
     */
    private Datum[] patternMatch(int offset) {
        double minDelta = Double.POSITIVE_INFINITY;
        int minDeltaIndex = 0;

        // get a copy of the history in an array
        // history is the list of recorded enemy data
        // most recent first i.e. index 0 is the most recent
        Datum[] history = new Datum[JonBot.patternData.size()];
        for(int i=0; i<JonBot.patternData.size(); i++) {
            history[i] = JonBot.patternData.get(i);
        }

        // get the OFFSET most recent items of history
        // index 0 is the first item of the pattern,
        // going forward in time the higher the index
        // ex. [0 => first event chronologically, 1 => second event, ... offset => last event]
        Datum[] pattern = new Datum[offset];
        for(int i=offset-1, j=0; i>=0; i--, j++) {
            pattern[j] = history[i];
        }
        // all good above

        // loop over every event in history, starting at the first one after the pattern
        // with a buffer of OFFSET events to reserve for predictive data.
        // e.g. the most recent OFFSET events
        for(int i=offset*2; i<history.length-offset; i++) {
            double delta = 0;
            for(int j=0; j<offset; j++) {
                Datum patternItem = pattern[j];
                Datum sampleItem = history[i+(offset-j)];
                delta += Math.abs(sampleItem.headingDelta-patternItem.headingDelta)
                         + Math.abs(sampleItem.velocity-patternItem.velocity);
            }
            if(delta < minDelta) {
                minDelta = delta;
                minDeltaIndex = i;
            }
        }

        Datum[] prediction = new Datum[offset];
        int predictionStartIndex = minDeltaIndex - offset;
        for(int i=0, j=predictionStartIndex; i<offset; i++, j--) {
            prediction[i] = history[j];
        }

        return prediction;
    }


    /**
     * Uses the pattern of historic behavior of the enemy to replay that
     * behavior over the current situation, returning the absolute heading
     * to predicted enemy location such that firing right now with current
     * fire power, the bullet would intersect their future position.
     * @param position Point2D.Double
     * @param prediction Datum[]
     * @return Point2D.Double
     */
    private Point2D.Double simulatePosition(double heading, Point2D.Double position, Datum[] prediction) {
        this.predictedPath = new Point2D.Double[prediction.length+1];
        this.predictedPath[0] = position;
        int i = 1;
        for(Datum d : prediction) {
            heading += d.headingDelta;
            position = JonBot.project(position, heading, d.velocity);
            this.predictedPath[i] = position;
            i++;
        }
        return position;
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
        if((num = JonBot.patternData.size()) > MAX_HISTORY_ITEMS) {
            JonBot.patternData.remove(num-1);
        }
    }


    private Point2D.Double getEnemyPosition(ScannedRobotEvent e) {
        double distance = e.getDistance();
        double myAbsoluteBearing = this.getHeadingRadians() + e.getBearingRadians();

        return JonBot.project(new Point2D.Double(this.getX(), this.getY()), myAbsoluteBearing, distance);
    }


    @Contract(pure=true)
    private static Point2D.Double project(Point2D.Double sourcePoint, double angle, double length) {
        return new Point2D.Double(
                sourcePoint.x + Math.sin(angle) * length,
                sourcePoint.y + Math.cos(angle) * length
        );
    }


    /**
     * onScannedRobot: What to do when you see another robot
     */
    public void onScannedRobot(ScannedRobotEvent e) {
        this.firePower = this.calculateFirePower(e.getDistance());
        this.recordData(e);

        // if we've recorded enough data to pattern match
        if(JonBot.patternData.size() >= MIN_HISTORY_ITEMS) {
            Datum currentEnemyData = JonBot.patternData.get(0);
            double bulletSpeed = Rules.getBulletSpeed(this.firePower);
            int ticksToSimulate = (int)(currentEnemyData.distance/bulletSpeed);

            Datum[] prediction = this.patternMatch(ticksToSimulate);
            Point2D.Double enemyPosition = this.getEnemyPosition(e);

            Point2D.Double predictedPosition = this.simulatePosition(e.getHeading(), enemyPosition, prediction);
            this.lastPredictedLocation = predictedPosition;
            double predictedAbsoluteHeading = Math.atan2(predictedPosition.x-this.getX(), predictedPosition.y-this.getY());

            this.setTurnGunRightRadians(Utils.normalRelativeAngle(predictedAbsoluteHeading - this.getGunHeadingRadians()));
        } else {
            // just aim at the target if there's not enough data to predict.
            double myAbsoluteBearing = this.getHeadingRadians() + e.getBearingRadians();
            if(e.getDistance() > 50) {
                this.setTurnGunRightRadians(Utils.normalRelativeAngle(myAbsoluteBearing - getGunHeadingRadians() +
                        (e.getVelocity() * Math.sin(e.getHeadingRadians() - myAbsoluteBearing) / 13.0)));
            } else {
                this.setTurnGunRightRadians(this.getAdjustedBearingToEnemy(e, this.getGunHeadingRadians()));
            }
        }



        this.setTurnRadarRightRadians(this.getAdjustedBearingToEnemy(e, this.getRadarHeadingRadians()));

        double updatedPlayerHeading = this.getAdjustedBearingToEnemy(e, this.getHeadingRadians());
        double collisionAvoidedHeading = this.adjustHeadingAvoidCollisions(updatedPlayerHeading);
        this.setTurnRightRadians(collisionAvoidedHeading);

        fire(this.firePower);

        setAhead(30);
    }


    @Override
    public void onPaint(Graphics2D g) {
        g.setColor(Color.red);
        if(this.lastPredictedLocation != null) {
            g.drawLine((int)this.getX(), (int)this.getY(), (int)this.lastPredictedLocation.x, (int)this.lastPredictedLocation.y);
        }
        if(this.predictedPath != null) {
            g.setColor(Color.blue);
            for(Point2D.Double point: this.predictedPath) {
                g.fillOval((int)point.x, (int)point.y, 6, 6);
            }
        }
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
