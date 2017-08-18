/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package de.tum.nst.jaer.projects.ritt;

import ch.unizh.ini.jaer.projects.rbodo.opticalflow.AbstractMotionFlow;
import ch.unizh.ini.jaer.projects.rbodo.opticalflow.LocalPlanesFlow;
import ch.unizh.ini.jaer.projects.rbodo.opticalflow.LucasKanadeFlow;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import java.util.Iterator;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.ApsDvsEventPacket;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.orientation.ApsDvsMotionOrientationEvent;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;
import net.sf.jaer.util.DrawGL;

/**
 * Computes the time to contact based on optical flow estimation. Algorithm is
 * based on the method presented in X. Clady et al. "Asynchronous visual
 * event-based time-to-contact", Frontiers in Neuroscience, 2014. Initially FOE
 * (Focus of Expansion) is computed from optical flow orientation. TTC (time to
 * contact) is computed form optical flow magnitude.
 *
 * @author rittk
 */
@Description("Estimation of time to contact with object in path of own motion")
@DevelopmentStatus(DevelopmentStatus.Status.Experimental)
public class TimeToContact extends EventFilter2D implements FrameAnnotater {

    //_______________________General variables__________________________________
    // Basic event information. Type is event polarity value, 0=OFF and 1=ON
    private int x, y, ts, type;

    // Orientation Event information: optical flow values
    private double vx, vy;

    // chip sizes
    private int sizex, sizey;

    //____________________Variables for TTC algortihm___________________________
    // indices for pixels
    int ix, iy;

    //____________________Variables for FOE algorithm___________________________
    // Matrix containing probability of FOE, last time updated
    private double[][] mProb;
    private int[][] mTime;
    // current maximum probability
    private double mProbMax;
    // coordinates of maximum probability: location of FOE
    private int foeX, foeY;

    //  ______filtering incoming events
    // valid flow vector angle in degrees, +- vector to center from current pixel
    private final double validAngle = 45 * Math.PI / 180.0;
    // center coordinates of current sensor
    private int centerX, centerY;
    // radius of evaluated pixels around current foe for probability update
    private int updateR;

    //_______________Variables handling optical flow estimation_________________
    private EventPacket flowPacket;
    private final AbstractMotionFlow flowFilter;

    //____________________Variables for user handling___________________________
    private boolean displayFOE = getBoolean("displayFOE", true);
    private boolean displayRawInput = getBoolean("displayRawInput", true);
    private float tProb = getFloat("tProb", 1000f);

    //_______________Variables for timing and performance_______________________
    private int filteredOutSpeed = 0;  // count events that are being omitted
    private int filteredOutPre = 0;
    private int filteredOutCentral = 0;

    public TimeToContact(AEChip chip) {
        super(chip);

        // configure enclosed filter that creates optical flow events
        flowFilter = new LocalPlanesFlow(chip);//new LucasKanadeFlow(chip); //TODO: make adaptable to other optical flow algos
        flowFilter.setDisplayRawInput(false); // to pass flow events not raw in        
        setEnclosedFilter(flowFilter);

        resetFilter();

        setPropertyTooltip("Focus of Expansion", "tProb", "time constant [microsec]"
                + " for probability decay of FOE position");
        setPropertyTooltip("Focus of Expansion", "displayFOE", "shows the estimated focus of expansion (FOE)");
        setPropertyTooltip("View", "displayRawInput", "shows the input events, instead of the motion types");

    }

    @Override
    synchronized public EventPacket filterPacket(EventPacket in) {
        if (!filterEnabled) {
            return in; // do this check to avoid always running filter
        }
        if (enclosedFilter != null && enclosedFilter.isFilterEnabled()) {
            flowPacket = enclosedFilter.filterPacket(in); // you can enclose a filter in a filter
        }

        Iterator i = null;
        i = ((EventPacket) flowPacket).inputIterator();

        while (i.hasNext()) {
            Object o = i.next();
            if (o == null) {
                log.warning("null event passed in, returning input packet");
                return in;
            }

            ApsDvsMotionOrientationEvent ein = (ApsDvsMotionOrientationEvent) o;
            if (ein.speed == 0f) {
                filteredOutSpeed++;
                continue;
            }
            // reset winning probability to 0
            mProbMax = 0;

            // get current event information
            x = ein.x;
            y = ein.y;
            ts = ein.timestamp;
            type = ein.type;
            vx = ein.velocity.x;
            vy = ein.velocity.y;

            if (getRelAngle(x - centerX, y - centerY, vx, vy) > validAngle) {
                ein.setFilteredOut(true);
                filteredOutCentral++;
                continue;
            }

            // FOE calculation (cf. Clady 2014)
            //for (ix = 0; ix < sizex; ++ix) {
            //for (iy = 0; iy < sizey; ++iy) {
            //ensure boundaries of pixels validated do not violate pixel dimensions
            int lbx = foeX - updateR;
            int ubx = foeX + updateR;
            if (lbx < 0) {
                lbx = 0;
            }
            if (ubx > sizex) {
                ubx = sizex;
            }
            for (ix = lbx; ix < sizex; ++ix) {
                int lby = (int) Math.round( foeY - Math.sqrt(updateR*updateR-(ix-foeX)*(ix-foeX)) );
                int uby = (int) Math.round( foeY + Math.sqrt(updateR*updateR-(ix-foeX)*(ix-foeX)) );
                if (lby < 0) {
                    lby = 0;
                }
                if (uby > sizey) {
                    uby = sizey;
                }
                for (iy = lby; iy < uby; ++iy) {
                    // check if current pixel is on the negative semi half plane
                    if ((x - ix) * vx + (y - iy) * vy > 0) {
                        mProb[ix][iy] += 1;
                        mTime[ix][iy] = ts;
                    } else {
                        mProb[ix][iy] = mProb[ix][iy] * Math.exp(-(ts - mTime[ix][iy]) / tProb);
                    }

                    if (mProb[ix][iy] > mProbMax) {
                        mProbMax = mProb[ix][iy];
                        foeX = ix;
                        foeY = iy;
                    }
                }
            }
        }// end while(i.hasNext())

        System.out.println("speed = " + filteredOutSpeed + " central = " + filteredOutCentral);
        return isDisplayRawInput() ? in : flowPacket;
    }

    /**
     * Returns the relative unsigned angle between vectors v1(x1,y1) & v2(x2,y2)
     */
    /**
     * Returns the relative unsigned angle between vectors v1(x1,y1) & v2(x2,y2)
     */
    private double getRelAngle(double x1, double y1, double x2, double y2) {
        double normalize1 = 1 / Math.sqrt(x1 * x1 + y1 * y1);
        double normalize2 = 1 / Math.sqrt(x2 * x2 + y2 * y2);
        return Math.acos(normalize1 * normalize2 * (x1 * x2 + y1 * y2));
    }

    /**
     * should reset the filter to initial state
     */
    @Override
    public final synchronized void resetFilter() {
        sizex = chip.getSizeX();
        sizey = chip.getSizeY();
        mProb = new double[sizex][sizey];
        mTime = new int[sizex][sizey];
        centerX = sizex / 2 - 1;
        centerY = sizey / 2 - 1;
    }

    /**
     * Should allocate and initialize memory. it may be called when the chip
     * e.g. size parameters are changed after creation of the filter.
     */
    @Override
    public final void initFilter() {
        resetFilter();
    }

    public boolean isDisplayFOE() {
        return displayFOE;
    }

    public void setDisplayFOE(boolean displayFOE) {
        this.displayFOE = displayFOE;
        putBoolean("displayFOE", displayFOE);
    }

    public boolean isDisplayRawInput() {
        return displayRawInput;
    }

    public void setDisplayRawInput(boolean displayRawInput) {
        this.displayRawInput = displayRawInput;
        putBoolean("displayRawInput", displayRawInput);
    }

    public float getTProb() {
        return this.tProb;
    }

    public void setTProb(final float tProb_) {
        this.tProb = tProb_;
        putFloat("tProb", tProb_);
    }

    @Override
    public void annotate(GLAutoDrawable drawable) {
        if (isDisplayFOE()) {
            if (!isFilterEnabled()) {
                return;
            }
            GL2 gl = drawable.getGL().getGL2();
            if (gl == null) {
                return;
            }
            checkBlend(gl);
            gl.glPushMatrix();
            DrawGL.drawCircle(gl, foeX, foeY, 4, 15);
            gl.glPopMatrix();
        }
    }

}
