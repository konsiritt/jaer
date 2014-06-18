/* DvsOrientationFilter.java
 *
 * Created on November 2, 2005, 8:24 PM */

package net.sf.jaer.eventprocessing.label;

import net.sf.jaer.event.orientation.DvsOrientationEvent;
import net.sf.jaer.event.orientation.BinocularOrientationEvent;
import net.sf.jaer.event.orientation.OrientationEventInterface;
import java.util.logging.Level;
import net.sf.jaer.chip.*;
import net.sf.jaer.event.*;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;

/** Computes simple-type orientation-tuned cells.                           <br>
 * <ul>
 * <li>SWITCH: multiOriOutputEnabled:                                       <br>
 *     {false} = WTA mode, meaning only max 1 orientation per event         <br>
 *     {true}  = any orientation that passes coincidence threshold 
 *               (so multiple per event possible)</li>                      <br>
 * <li>SWITCH: oriHistoryEnabled:                                           <br>
 *     {false} = orientations are generated just based on current events    <br>
 *     {true}  = Previous orientations at this location are used to 
 *               make generation of similar orientations easier and 
 *               inhibit very different orientations. This can be 
 *               understood as contour enhancement.</li>                    <br>
 * <li>SWITCH: useAverageDtEnabled:                                         <br>
 *     {false} = the maximum temporal difference per orientation is 
 *               used as coincidence measure.                               <br>
 *     {true}  = the average over all cells in the receptive field 
 *               per orientation is used as coincidence measure.</li>
 * </ul><p>
 * Orientation type output takes values 0-3;                                <br>
 * 0 is a horizontal edge (0 deg),                                          <br>
 * 1 is an edge tilted up and to right (rotated CCW 45 deg),                <br>
 * 2 is a vertical edge (rotated 90 deg),                                   <br>
 * 3 is tilted up and to left (rotated 135 deg from horizontal edge).
 * <p>
 * The filter takes either PolarityEvents or BinocularEvents to create 
 * DvsOrientationEvent or BinocularOrientationEvents.
 * @author tobi/phess */
@Description("Detects local orientation by spatio-temporal correlation for DVS sensors")
@DevelopmentStatus(DevelopmentStatus.Status.Experimental)
public class DvsOrientationFilter extends AbstractOrientationFilter{
    //TODO: The oriHistoryMap is still not completely bias-free.
    //      The values for orientation range from 0-3 and each time we update
    //      the oriHistory map we adjust the value of the history slightly towards 
    //      the currently detected orientation. However, as the directions are
    //      represented by numbers 0 to 3 we are not implementing periodic
    //      boundary conditions. In fact if the oriHistoryMap has a value of 3
    //      for a given pixel and we constantly see values of 0 the HistoryMap
    //      will update from the direction 'up-left' towards 'horizontal' not
    //      directly, but by taking the path 'up-left'-->'vertical'-->'up-right'-->'horizontal'
    //      although the best path would be 'up-left'-->'horizontal' ...
    //      This fact gives a bias towards orientations 1 and 2 and yields 
    //      fewer outputs of orientations 0 and 3...
    
    private boolean isBinocular;
    
    /** Creates a new instance of DvsOrientationFilter
     * Most of the initialization is done in the Abstract super of this class.
     * @param chip */
    public DvsOrientationFilter (AEChip chip){
        super(chip);
        //Tooltips and Properties are defined in the AbstractOrientationFilter.
    }

    /** filters in to getOutputPacket(). 
     * This filter can be used as FILTER (i.e. the number of events in the 
     * outputPacket can be less than the number of input events) or as LABELER
     * (i.e. the number of events is exactly the same) based on the flag
     * 'passAllEvents'
     * @param in input events can be null or empty.
     * @return the processed events, may be fewer in number. */
    @Override synchronized public EventPacket<?> filterPacket (EventPacket<?> in){
        if ( enclosedFilter != null ) in = enclosedFilter.filterPacket(in);
        if ( in.getSize() == 0 ) return in;

        Class inputClass = in.getEventClass();
        if ( inputClass == PolarityEvent.class) {
            isBinocular = false;
            checkOutputPacketEventType(DvsOrientationEvent.class);
        } else if( inputClass == BinocularEvent.class ) {
            isBinocular = true;
            checkOutputPacketEventType(BinocularOrientationEvent.class);
        } else { //Neither Polarity nor Binocular Event --> Wrong class used!
            log.log(Level.WARNING, "wrong input event class {0} in the input packet {1}, disabling filter", new Object[]{inputClass, in});
            setFilterEnabled(false);
            return in;
        }

        EventPacket outputPacket = getOutputPacket();
        OutputEventIterator outItr = outputPacket.outputIterator();

        int sizex = chip.getSizeX() - 1;
        int sizey = chip.getSizeY() - 1;

        oriHist.reset();
        checkMaps(in);

        // for each event write out an event of an orientation type if 
        // there have also been events within past dt along this 
        // type's orientation of the same retina polarity
        for ( Object ein:in ){
            PolarityEvent e = (PolarityEvent)ein;
            
            if(e.isSpecial())continue;
            
            int    x = e.x >>> subSampleShift;
            int    y = e.y >>> subSampleShift;
            int type = e.getType();
            
            //TODO: Is this check really necessary? Should those special events being marked 'special'? (They would have already being catched above)
            if(type >= NUM_TYPES || e.x < 0||e.y < 0) {
                continue;  // tobi - some special type like IMU sample
            }

            /* (Peter Hess) monocular events use eye = 0 as standard. therefore some arrays will waste memory, because eye will never be 1 for monocular events
             * in terms of performance this may not be optimal. but as long as this filter is not final, it makes rewriting code much easier, because
             * monocular and binocular events may be handled the same way. */
            int eye = 0;
            if(isBinocular && (( (BinocularEvent)ein ).eye == BinocularEvent.Eye.RIGHT)){
                eye = 1;
            }
            if ( eye == 1 ){
                type = type << 1;
            }
            lastTimesMap[x][y][type] = e.timestamp;

            // For each orientation and position in the receptive field compute
            // the time to last event of the same type.
            // TODO: Currently this is wrong as soon as subsampling is enabled.
            //       The offset is not subsampled, so when subsampling, the 
            //       offsets are still computed in the 'unsampled' space.
            int xx, yy;
            for ( int ori = 0 ; ori < NUM_TYPES ; ori++ ){
                // 'd' is vector of spatial offsets for this orientation. It is
                // precomputed for performance. Each element of 'd' is a offset
                // direction to an element of the RF.
                Dir[] d = offsets[ori]; 
                for ( int i = 0 ; i < d.length ; i++ ){ 
                    xx = x + d[i].x;
                    yy = y + d[i].y;
                    
                    if ( xx < 0 || xx > sizex ) continue;
                    if ( yy < 0 || yy > sizey ) continue; // indexing out of array
                    
                    dts[ori][i] = e.timestamp - lastTimesMap[xx][yy][type];
                }
            }
            
            //Compute the average or maximum time to last event within RF
            if ( useAverageDtEnabled ){
                // <editor-fold defaultstate="collapsed" desc="--compute the average dt's in each direction--">
                for ( int ori = 0 ; ori < NUM_TYPES ; ori++ ){
                    oridts[ori] = 0; 
                    oriDecideHelper[ori] = 0;
                    
                    int count = 0;
                    int[] dtList = new int[rfSize];
                    for ( int k = 0 ; k < rfSize ; k++ ){
                        int dt = dts[ori][k];
                        if ( dt > dtRejectThreshold ){
                            //TODO: bbeyer: There is a small bug here that I could not
                            // figure out. Using the orientation sample data from the jAER
                            // project page and measuring how often events where rejected
                            // due to dtRejectThreshold I could reliably find, that
                            // diagonal orientations (ori 1 and 3) get rejected 11% more
                            // often than horizontal and vertical orientations.
                            // At first I thought that this might be due to the euclidian distance
                            // to diagonal pixels being longer than to horizontal/vertical pixels.
                            // But since we are concerned about orientation not motion here this should
                            // not matter!?
                            // Basically I dont know why diagonal orientations get systematicallty rejected
                            // more often, but I am pretty certain that it should not be the case, given 
                            // a well diversified sample...
                            continue; // we're averaging delta times; this rejects outliers
                        }
                        oridts[ori] += dt; // average dt
                        dtList[k] = dt;
                        count++;
                    }
                    if ( count > 0 ){
                        oridts[ori] /= count; // normalize by RF size
                        for(int k =0;k<rfSize;k++) {
                            if(dtList[k]>0) dtList[k] -= oridts[ori];
                            oriDecideHelper[ori] += (dtList[k]*dtList[k]);
                        }
                        if(oriDecideHelper[ori]<0)oriDecideHelper[ori] = Integer.MAX_VALUE; //Happens when dtList[k]^2 is larger than maxint or the sum exceeds maxint.
                        oriDecideHelper[ori] /= count; //biased estimator of variance
                        
                    } else {
                        // no samples, all outside outlier rejection threshold
                        oridts[ori] = Integer.MAX_VALUE;
                        oriDecideHelper[ori] = Integer.MAX_VALUE;
                    }
                }
                // </editor-fold>
            } else { // use max dt
                //bbeyer: I dont really see a situation in which the maxDt method
                // would be preferred over the averageDt method. When is this useful?
                // I find the maxDt method to be unreliable and unmeaningful, wheras
                // I can understand the idea behind the averageDt method.
                // <editor-fold defaultstate="collapsed" desc="--compute the max dt in each direction--">
                for ( int ori = 0 ; ori < NUM_TYPES ; ori++ ){
                    oridts[ori] = Integer.MIN_VALUE;
                    oriDecideHelper[ori] = Integer.MIN_VALUE;

                    for ( int k = 0 ; k < rfSize ; k++ ){  
                        // iterate over RF and find maxdt to previous events, final orientation will be that orientation that has minimum maxdt
                        // this has problem that pixels that do NOT fire an event still contribute a large dt from previous edges
                        if ( dts[ori][k] > dtRejectThreshold ){
                            continue; // reject old timestamps to better detect edges
                        }
                        if(dts[ori][k] > oridts[ori]){
                            oriDecideHelper[ori] = oridts[ori];// we need this if two oridts are equal
                            oridts[ori] = dts[ori][k];
                        }
                        //readability^^
                        //maxdts[ori] = dts[ori][k] > oridts[ori] ? dts[ori][k] : oridts[ori]; // max dt to neighbor
                    }
                }
                // </editor-fold>
            }

            if ( !multiOriOutputEnabled ){
                // <editor-fold defaultstate="collapsed" desc="--WTA to find the one best orientation per event--">
                // here we do a WTA, only 1 event max gets generated in optimal 
                // orienation IFF is also satisfies coincidence timing requirement

                // now find min of these, this is most likely orientation, iff this time is also less than minDtThresholdUs
                int mindt = minDtThresholdUs, decideHelper = 0, dir = -1;
                for ( int ori = 0 ; ori < NUM_TYPES ; ori++ ){
                    if ( oridts[ori] < mindt ){
                        mindt = oridts[ori];
                        decideHelper = oriDecideHelper[ori];
                        dir = ori;
                    } else if(oridts[ori] == mindt){
                        // <editor-fold defaultstate="collapsed" desc="--COMMENT--">
                        //Before only the check above ( oridts[ori] < mindt )
                        // was performed. This however was biased towards 
                        // horizontal directions, as if the oridts[ori] of 
                        // 'later' orientations was equal to an earlier one
                        // then the final orientation would still be the first 
                        // to accour. As we always loop from horizontal to
                        // up-left the up-left orientation had a dissadvantage.
                        // The oriDecideHelper is either the second largest time
                        // if the MAXdt method is used, or the estimator of the
                        // standard deviation if the average method is used.
                        //  This still has a very small bias, as if the second
                        //  decicion is also equal (oriDecideHelper == decideHelper)
                        //  we chose the new orientation over the old one.
                        //  THis is much more unlikely though.
                        // </editor-fold>
                        if(oriDecideHelper[ori] <= decideHelper){
                            mindt = oridts[ori];
                            dir = ori;
                        }
                    }
                }
                
                if ( dir == -1 ){ // didn't find a good orientation(meaning oridts[ori] has been larger than minDtThresholdUs for all ori)
                    if ( passAllEvents ) writeOutput(outItr , e , false , (byte)0);
                    continue;
                }
                
                if ( oriHistoryEnabled ){
                    //We only let the orientation pass if it is within some
                    // agreement with the past orientations we found at this 
                    // particular spot. If it is too different, we ignore it
                    // and dont count it as a valid orientation.
                    
                    // update lowpass orientation map
                    float f = oriHistoryMap[x][y];
                    if(f == -1f) {
                        //If we initialize to 0 we have a big bias towards 
                        // horizontal direction. Hence, we initialize the 
                        // array to '-1' and check for this. If we find a '-1'
                        // this means this location has never had a orientation
                        // and we initialize it with the found location.
                        // This should guarantee handling the orientations in a 
                        // unbiased fashion.
                        f = dir;
                    } 
                    f = ( 1 - oriHistoryMixingFactor ) * f + oriHistoryMixingFactor * dir;
                    oriHistoryMap[x][y] = f;

                    //fd is the distance between the orientation in the HistoryMap
                    // and the currently detected orientation.
                    float fd = f - dir;
                    final int halfTypes = NUM_TYPES / 2;
                    //The distance between orientation 0 (horizontal) and ori 3
                    // (up-left) is not equal to 0-3=3 but infact is just 1.
                    // There is one orientation between 0 and 3, hence we need
                    // to adjust here.
                    if ( fd > halfTypes ){
                        fd = fd - NUM_TYPES;
                    } else if ( fd < -halfTypes ){
                        fd = fd + NUM_TYPES;
                    }
                    if ( Math.abs(fd) > oriHistoryDiffThreshold ){
                        if ( passAllEvents ) writeOutput(outItr , e , false , (byte)0);
                        continue;
                    }
                }
                
                writeOutput(outItr , e , true , (byte)dir);
                oriHist.add(dir);
                // </editor-fold>
            } else {
                // <editor-fold defaultstate="collapsed" desc="--allow multiple orientations per event--">
                // here events are generated in oris that satisfy timing; there is no WTA
                // now write output cell iff all events along dir occur within minDtThresholdUs
                for ( int k = 0 ; k < NUM_TYPES ; k++ ){
                    if ( oridts[k] < minDtThresholdUs ){
                        writeOutput(outItr , e , true , (byte)k);
                        oriHist.add(k);
                    } else writeOutput(outItr , e , false , (byte)0);
                }
                // </editor-fold>
            }
        }

        return showRawInputEnabled ? in : getOutputPacket();
    }
    
    private void writeOutput(OutputEventIterator outItr, PolarityEvent e, boolean hasOrientation, byte orientation){
        OrientationEventInterface eout = (OrientationEventInterface) outItr.nextOutput();
        eout.copyFrom(e);
        eout.setHasOrientation(hasOrientation);
        eout.setOrientation(orientation);
    }
  
}
