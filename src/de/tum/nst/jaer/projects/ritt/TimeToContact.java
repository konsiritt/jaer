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
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;
import java.util.Vector;
import java.util.logging.Level;
import javax.swing.JFileChooser;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.ApsDvsEventPacket;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.orientation.ApsDvsMotionOrientationEvent;
import static net.sf.jaer.eventprocessing.EventFilter.log;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.graphics.FrameAnnotater;
import net.sf.jaer.util.DrawGL;
import net.sf.jaer.util.TobiLogger;

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
    private int ix, iy;
    // lower and upper bounds for pixel iteration
    private int lbx, ubx, lby, uby;
    // location of obstacle (first from rendering selection chip.getCanvas().getRenderer().getXsel()
    private int obstX, obstY;
    // distance between current event and foe estimate
    private int distPx, distPy;
    // distance threshold for discarding to far away events in pixels
    private int threshDist = getInt("threshDist", 30);
    // velocity threshold for discarding too slow events in [pixels/s]
    private float threshVel = getFloat("threshVel",0.5f);
    // time threshold for discarding too long away ttc times [s]
    private float threshTTC = getFloat("threshTTC",10f);
    // current (for this event) ttc estimate in [s]
    private double currTTC;
    // overall lowest ttc estimate in [s]
    private double ttc = 0;
    // update weight for EMA 
    private float alphaTTC = getFloat("alphaTTC",0.033f);;
    // amount of initial updates
    private int countInitTTC;
    // threshold for initialization of ttc estimate
    private int threshInitTTC = 10; 

    //____________________Variables for FOE algorithm___________________________
    // Matrix containing probability of FOE, last time updated
    private double[][] mProb;
    private int[][] mTime;
    // current maximum probability
    private double mProbMax;
    // coordinates of maximum probability: location of FOE
    private int foeX, foeY;
    // time constant [microsec] for probability decay of FOE position
    private float tProb = getFloat("tProb", 1000f);
    // class that handles logging of obtained FOE
    private TobiLogger foeEstimationLogger = null;

    //  ______filtering incoming events
    // radius of evaluated pixels around current foe for probability update
    private int updateR = getInt("updateR", 50);
    // turn diverging field around center filter on/off
    private boolean centralFilter = getBoolean("centralFilter", true);
    // valid flow vector angle in degrees, +- vector to center from current pixel
    private float validAngle = getFloat("validAngle", (float) (120) );// * Math.PI / 180.0) );    
    // center coordinates of current sensor
    private int centerX, centerY;    

    // _______ground truth related
    // variable that gives availability of ground truth data
    private boolean importedGTfromFile = false;
    // ground truth values:
    private Vector<Double> gtTime;
    private Vector<Float> gtFoeX, gtFoeY;

    //_______________Variables handling optical flow estimation_________________
    private EventPacket flowPacket;
    private final AbstractMotionFlow flowFilter;

    //____________________Variables for user handling___________________________
    private boolean displayFOE = getBoolean("displayFOE", true);
    private boolean displayRawInput = getBoolean("displayRawInput", true);

    //_______________Variables for timing and performance_______________________
    private int filteredOutSpeed = 0;  // count events that are being omitted
    private int filteredOutPre = 0;
    private int filteredOutCentral = 0;

    public TimeToContact(AEChip chip) {
        super(chip);

        // configure enclosed filter that creates optical flow events
        flowFilter = new LucasKanadeFlow(chip); //new LocalPlanesFlow(chip);// TODO: make adaptable to other optical flow algos
        flowFilter.setDisplayRawInput(false); // to pass flow events not raw in        
        setEnclosedFilter(flowFilter);

        resetFilter();

        setPropertyTooltip("importGTfromFile", "imports text file to load ground truth FOE data");
        setPropertyTooltip("resetGroundTruth", "Resets the ground truth focus of expansion loaded from file.");
        setPropertyTooltip("startLoggingFOE", "logs the estimated FOE position");
        setPropertyTooltip("stopLoggingFOE", "stops logging the estimated FOE position");
        setPropertyTooltip("Time to Contact", "threshDist", "distance threshold for events being considered in ttc estimation (in pixels)");
        setPropertyTooltip("Time to Contact", "threshVel", "velocity threshold for events being considered in ttc estimation (in pixels/s)");
        setPropertyTooltip("Time to Contact", "threshTTC", "ttc threshold for events being considered in ttc estimation (s)");
        setPropertyTooltip("Time to Contact", "alphaTTC", "averaging ttc threshold for per event contribution");
        setPropertyTooltip("Focus of Expansion", "tProb", "time constant [microsec]"
                + " for probability decay of FOE position");
        setPropertyTooltip("Focus of Expansion", "updateR", "radius [pixels] around current FOE where probability is updated");        
        setPropertyTooltip("Focus of Expansion", "centralFilter", "turn filter for diverging field around center on/off");
        setPropertyTooltip("Focus of Expansion", "validAngle", "valid angle for central filter. flow events with angle between"
                + " distance to center and flow direction greater than validAngle are discarded");
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

        //temporary counter
        int tempCountTtcEstimates = 0;
        //temporary package average
        double tempPackageAvgTTC = 0;
        
        // loop the events in the package
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

            int distXcenter = x - centerX;
            int distYcenter = y - centerY;
            if ( false && centralFilter && Math.sqrt(distXcenter*distXcenter + distYcenter*distYcenter)>updateR 
                    && getRelAngle(distXcenter, distYcenter, vx, vy) > validAngle*Math.PI/180) {
                ein.setFilteredOut(true);
                filteredOutCentral++;
                continue;
            }

            // FOE calculation (cf. Clady 2014)
            // ensure boundaries of pixels validated do not violate pixel dimensions            
            lbx = centerX - updateR;
            ubx = centerX + updateR;
            if (lbx < 0) {
                lbx = 0;
            }
            if (ubx > sizex) {
                ubx = sizex;
            }
            for (ix = lbx; ix < ubx; ++ix) {
                lby = (int) Math.round(centerY - Math.sqrt(updateR * updateR - (ix - centerX) * (ix - centerX)));
                uby = (int) Math.round(centerY + Math.sqrt(updateR * updateR - (ix - centerX) * (ix - centerX)));
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
                        mProb[ix][iy] = mProb[ix][iy] * Math.exp(-Math.abs(ts - mTime[ix][iy]) / tProb);
                    }

                    if (mProb[ix][iy] > mProbMax) {
                        mProbMax = mProb[ix][iy];
                        foeX = ix;
                        foeY = iy;
                    }
                }
            }
        
            // TTC calculation (cf. Clady 2014)
            // for now, no obstacle detection, rather obstacle selection
            //obstX = chip.getCanvas().getRenderer().getXsel();
            //obstY = chip.getCanvas().getRenderer().getYsel();
            int currentGTIndex = binarySearch(ts * 1e-6);
            
            distPx = x - Math.round( gtFoeX.get(currentGTIndex) );//foeX;
            distPy = y - Math.round(gtFoeY.get(currentGTIndex));//foeY;
            if (threshDist > Math.sqrt(distPx * distPx + distPy * distPy) //only include events in circle of threshDist pixels around the foe
                    && Math.sqrt(distPx * distPx + distPy * distPy) > 10 //do not include events too close to the foe
                    && threshVel < Math.sqrt(vx * vx + vy * vy) //only include flow events with sufficiently large flow velocity
                    && getRelAngle(distPx, distPy, vx, vy) < validAngle * Math.PI / 180  //only include flow events diverging from foe
                    && vy < 0 && distPy < 0 ) // hack: only include flow events below the foe
            {
                currTTC = 0.5 * (distPx / vx + distPy / vy);
                if (currTTC < threshTTC && currTTC > 0) {
                    if (ttc == 0) { //initialize
                        ttc = currTTC;
                    } else if (countInitTTC < threshInitTTC) { //initialize for threshInitTTC estimates
                        ttc = (countInitTTC * ttc + currTTC) / ++countInitTTC;
                    } else {
                        ttc += alphaTTC * (currTTC - ttc);
                        tempPackageAvgTTC = (tempPackageAvgTTC * tempCountTtcEstimates + currTTC) / ++tempCountTtcEstimates;
                    }
                } else {
                    ein.setFilteredOut(true);
                }
            } else {
                ein.setFilteredOut(true);
            }

        }// end while(i.hasNext())               

        log.info("Contributing ttc est: " + tempCountTtcEstimates + " overall ttc est: " + ttc + "per packet ttc: " + tempPackageAvgTTC);
        
        // if logging is turned on, then foe estimation results are logged to file after every packet
        if (foeEstimationLogger != null && foeEstimationLogger.isEnabled()) {
            logFoeData();
        }
        
        //log.info("current foe winner "+foeX+", "+foeY+" with probability = "
        //+mProb[foeX][foeY]+ " time since last update = " +(ts - mTime[foeX][foeY]));
        //int currentGTIndex = binarySearch(ts*1e-6);
        //System.out.println("speed = " + filteredOutSpeed + " central = " + filteredOutCentral + " foeX = " + foeX + " foeY = " + foeY
        //+ " corresponding gt foeX=" + gtFoeX.get(currentGTIndex) + " foeY=" +gtFoeY.get(currentGTIndex) + " at time ts=" + ts
        //+ " with index " + currentGTIndex);
        return isDisplayRawInput() ? in : flowPacket;
    }
    
    /**
     * logs relevant FOE information to file
     */
    private void logFoeData() {
        String s;
        if (importedGTfromFile) {
            int currentGTIndex = binarySearch(ts * 1e-6);
            s = String.format("%d %d %d %g %g %g %g", ts, foeX, foeY, gtFoeX.get(currentGTIndex), gtFoeY.get(currentGTIndex), mProb[foeX][foeY], ttc);
        } else {
            s = String.format("%d %d %d %g %g", ts, foeX, foeY, mProb[foeX][foeY], ttc);
        }
        foeEstimationLogger.log(s);
    }

    /**
     * Returns the relative unsigned angle between vectors v1(x1,y1) & v2(x2,y2)
     */
    private synchronized double getRelAngle(double x1, double y1, double x2, double y2) {
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
        foeX = centerX;
        foeY = centerY;
        countInitTTC = 0;
    }

    /**
     * Should allocate and initialize memory. it may be called when the chip
     * e.g. size parameters are changed after creation of the filter.
     */
    @Override
    public final void initFilter() {
        resetFilter();
    }

    // Allows importing two 2D-arrays containing the x-/y- components of the 
    // motion flow field used as ground truth.
    synchronized public void doImportGTfromFile() {
        JFileChooser chooser = new JFileChooser();
        chooser.setDialogTitle("Choose ground truth file");
        chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
        chooser.setMultiSelectionEnabled(false);

        gtTime = new Vector<Double>();
        gtFoeX = new Vector<Float>();
        gtFoeY = new Vector<Float>();
        BufferedReader reader = null;
        File file = null;
        if (chooser.showOpenDialog(chip.getAeViewer().getFilterFrame()) == JFileChooser.APPROVE_OPTION) {
            try {
                file = new File(chooser.getSelectedFile().getPath());

                log.info("Imported ground truth file");
                reader = new BufferedReader(new FileReader(file));
                String text = null;

                while ((text = reader.readLine()) != null) {
                    String[] columns = text.split(" ");
                    if (columns[0].matches("#")) {
                        log.info("commented line removed");
                        continue;
                    }
                    gtTime.add(Double.parseDouble(columns[0]));
                    gtFoeX.add((float) Double.parseDouble(columns[22]));
                    gtFoeY.add((float) Double.parseDouble(columns[23]));
                }
                importedGTfromFile = true;
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                log.log(Level.SEVERE, null, e);
            } finally {
                try {
                    if (reader != null) {
                        reader.close();
                    }
                } catch (IOException e) {
                }
            }
        }
        log.info("import of ground truth done, data with times from " + gtTime.firstElement() + " to " + gtTime.lastElement());
    }

    void resetGroundTruth() {
        importedGTfromFile = false;
        gtTime = null;
        gtFoeX = null;
        gtFoeY = null;
    }

    synchronized public void doResetGroundTruth() {
        resetGroundTruth();
    }
    
    synchronized public void doStartLoggingFOE() {
        if (foeEstimationLogger != null && foeEstimationLogger.isEnabled()) {
            log.info("logging already started");
            return;
        }
        String filename = null, filepath = null;
        final JFileChooser fc = new JFileChooser();
        fc.setCurrentDirectory(new File(getString("lastFile", System.getProperty("user.dir"))));  // defaults to startup runtime folder
        fc.setFileSelectionMode(JFileChooser.FILES_ONLY);
        fc.setSelectedFile(new File(getString("lastFile", System.getProperty("user.dir"))));
        fc.setDialogTitle("Select folder and base file name for the logged foe estimation data");
        int ret = fc.showOpenDialog(chip.getAeViewer() != null && chip.getAeViewer().getFilterFrame() != null ? 
                chip.getAeViewer().getFilterFrame() : null);
        if (ret == JFileChooser.APPROVE_OPTION) {
            File file = fc.getSelectedFile();
            putString("lastFile", file.toString());
            foeEstimationLogger = new TobiLogger(file.getPath(), "Focus of expansion estimation results from optical flow");
            foeEstimationLogger.setNanotimeEnabled(false);
            if (importedGTfromFile) {
                foeEstimationLogger.setHeaderLine("system_time(ms) timestamp(us) foeX foeY foeXgt foeYgt winningProb ttc");
            } else {
                foeEstimationLogger.setHeaderLine("system_time(ms) timestamp(us) foeX foeY winningProb ttc");
            }
            foeEstimationLogger.setEnabled(true);
        } else {
            log.info("Cancelled logging motion vectors");
        }
    }
    
    synchronized public void doStopLoggingFOE() {
        if (foeEstimationLogger == null) {
            return;
        }
        foeEstimationLogger.setEnabled(false);
        foeEstimationLogger = null;
    }

    public int getUpdateR() {
        return this.updateR;
    }

    public void setUpdateR(final int updateR_) {
        this.updateR = updateR_;
        putInt("updateR", updateR_);
    }
    
    public float getTProb() {
        return this.tProb;
    }

    public void setTProb(final float tProb_) {
        this.tProb = tProb_;
        putFloat("tProb", tProb_);
    }
    
    public int getThreshDist() {
        return this.threshDist;
    }

    public void setThreshDist(final int threshDist_) {
        this.threshDist = threshDist_;
        putInt("threshDist", threshDist_);
    }
    
    public float getThreshVel() {
        return this.threshVel;
    }

    public void setThreshVel(final float threshVel_) {
        this.threshVel = threshVel_;
        putFloat("threshVel", threshVel_);
    }
    
    public float getThreshTTC() {
        return this.threshTTC;
    }

    public void setThreshTTC(final float threshTTC_) {
        this.threshTTC = threshTTC_;
        putFloat("threshTTC", threshTTC_);
    }
    
    public float getAlphaTTC() {
        return this.alphaTTC;
    }

    public void setAlphaTTC(final float alphaTTC_) {
        this.alphaTTC = alphaTTC_;
        putFloat("alphaTTC", alphaTTC_);
    }
    
    public boolean isCentralFilter() {
        return centralFilter;
    }

    public void setCentralFilter(boolean centralFilter_) {
        this.centralFilter = centralFilter_;
        putBoolean("centralFilter", centralFilter_);
    }

    public float getValidAngle() {
        return this.validAngle;
    }

    public void setValidAngle(final float ValidAngle_) {
        this.validAngle = ValidAngle_;
        putFloat("validAngle", ValidAngle_);
    }

    public boolean isDisplayFOE() {
        return displayFOE;
    }

    public void setDisplayFOE(boolean displayFOE_) {
        this.displayFOE = displayFOE_;
        putBoolean("displayFOE", displayFOE_);
    }

    public boolean isDisplayRawInput() {
        return displayRawInput;
    }

    public void setDisplayRawInput(boolean displayRawInput) {
        this.displayRawInput = displayRawInput;
        putBoolean("displayRawInput", displayRawInput);
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

        if (importedGTfromFile) {
            if (!isFilterEnabled()) {
                return;
            }
            GL2 gl = drawable.getGL().getGL2();
            if (gl == null) {
                return;
            }
            checkBlend(gl);
            gl.glPushMatrix();
            int currentGTIndex = binarySearch(ts * 1e-6);
            DrawGL.drawBox(gl, gtFoeX.get(currentGTIndex), gtFoeY.get(currentGTIndex), 4, 4, (float) (45 * Math.PI / 180));
            gl.glPopMatrix();
        }
    }

    // returns the index of the timestamp in GT data closest to current time
    private int binarySearch(double time) {
        if (!importedGTfromFile) {
            return 0;
        }

        if (time < gtTime.firstElement()) {
            return 0;
        }
        if (time > gtTime.lastElement()) {
            return gtTime.size() - 1;
        }

        int lo = 0;
        int hi = gtTime.size() - 1;

        while (lo <= hi) {
            int mid = (hi + lo) / 2;

            if (time < gtTime.get(mid)) {
                hi = mid - 1;
            } else if (time > gtTime.get(mid)) {
                lo = mid + 1;
            } else {
                return mid;
            }
        }
        // lo == hi + 1
        return (gtTime.get(lo) - time) < (time - gtTime.get(hi)) ? lo : hi;
    }

}
