/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package de.tum.nst.jaer.projects.ritt;

import ch.unizh.ini.jaer.projects.rbodo.opticalflow.AbstractMotionFlow;
import ch.unizh.ini.jaer.projects.rbodo.opticalflow.LucasKanadeFlow;
import java.util.Iterator;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.ApsDvsEventPacket;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.orientation.ApsDvsMotionOrientationEvent;
import net.sf.jaer.eventprocessing.EventFilter2D;

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
public class TimeToContact extends EventFilter2D {

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
    private double [][] mProb;
    private int [][] mTime;
    // current maximum probability
    private double mProbMax;
    // coordinates of maximum probability: location of FOE
    private int foeX, foeY;

    //_______________Variables handling optical flow estimation_________________
    private EventPacket flowPacket;
    private final AbstractMotionFlow flowFilter;

    //____________________Variables for user handling___________________________
    private boolean displayRawInput = getBoolean("displayRawInput", true);
    private float tProb = getFloat("tProb", 1000f);

    public TimeToContact(AEChip chip) {
        super(chip);

        // configure enclosed filter that creates optical flow events
        flowFilter = new LucasKanadeFlow(chip); //TODO: make adaptable to other optical flow algos
        flowFilter.setDisplayRawInput(false); // to pass flow events not raw in        
        setEnclosedFilter(flowFilter);

        resetFilter();

        setPropertyTooltip("Focus of Expansion", "tProb", "time constant [microsec]"
                + " for probability decay of FOE position");
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
            Object o=i.next();
             if (o == null) {
                log.warning("null event passed in, returning input packet");
                return in;
            }
             
            ApsDvsMotionOrientationEvent ein = (ApsDvsMotionOrientationEvent) o;
            if (ein.speed==0f) {
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
            
            // FOE calculation (cf. Clady 2014)
            for (ix = 0; ix < sizex; ++ix) {
                for (iy = 0; iy < sizey; ++iy) {
                    // check if current pixel is on the negative semi half plane
                    if ( (x-ix)*vx + (y-iy)*vy > 0 ) {
                        mProb[ix][iy] += 1;
                        mTime[ix][iy] = ts;                        
                    } else {
                        mProb[ix][iy] = mProb[ix][iy]*Math.exp(-(ts-mTime[ix][iy])/tProb);
                    }                  
                    
                    if (mProb[ix][iy] > mProbMax) {
                        mProbMax = mProb[ix][iy];
                        foeX = ix;
                        foeY = iy;
                    }
                }
            }
        
        }// end while(i.hasNext())

        return isDisplayRawInput() ? in : flowPacket;
    }

    /**
     * should reset the filter to initial state
     */
    @Override
    public synchronized void resetFilter() {
        sizex = chip.getSizeX();
        sizey = chip.getSizeY();
        mProb = new double [sizex][sizey];
        mTime = new int [sizex][sizey];
    }

    /**
     * Should allocate and initialize memory. it may be called when the chip
     * e.g. size parameters are changed after creation of the filter.
     */
    @Override
    public void initFilter() {
        resetFilter();
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

}
