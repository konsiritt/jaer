/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package de.tum.nst.jaer.projects.ritt;

import ch.unizh.ini.jaer.projects.rbodo.opticalflow.AbstractMotionFlow;
import ch.unizh.ini.jaer.projects.rbodo.opticalflow.LucasKanadeFlow;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.util.jama.Matrix;

/**
 * Computes the time to contact based on optical flow estimation.
 * Algorithm is based on the method presented in X. Clady et al. 
 * "Asynchronous visual event-based time-to-contact", Frontiers in
 * Neuroscience, 2014. Initially FOE (Focus of Expansion) is computed 
 * from optical flow orientation. TTC (time to contact) is computed form
 * optical flow magnitude. 
 * 
 * @author rittk
 */
@Description("Estimation of time to contact with object in path of own motion")
@DevelopmentStatus(DevelopmentStatus.Status.Experimental)
public class TimeToContact extends EventFilter2D {
    
    // Matrix containing probability of FOE, last time updated
    private Matrix mProb, mTime;
    
    // Variables handling optical flow estimation    
    private EventPacket flowPacket; 
    private final AbstractMotionFlow flowFilter;

    public TimeToContact(AEChip chip) {
        super(chip);
        
        // configure enclosed filter that creates optical flow events
        flowFilter = new LucasKanadeFlow(chip); //TODO: make adaptable
        flowFilter.setDisplayRawInput(false); // to pass flow events not raw in        
        setEnclosedFilter(flowFilter);
        
        //resetFilter();
    }
    
    @Override
    synchronized public EventPacket filterPacket(EventPacket in) {
        if (!filterEnabled) {
            return in; // do this check to avoid always running filter
        }
        if (enclosedFilter != null) {
            flowPacket = enclosedFilter.filterPacket(in); // you can enclose a filter in a filter
        }
        

        return in;
    }

    /**
     * should reset the filter to initial state
     */
    @Override
    public synchronized void resetFilter() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    /**
     * Should allocate and initialize memory. it may be called when the chip
     * e.g. size parameters are changed after creation of the filter.
     */
    @Override
    public void initFilter() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    
}
