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
    // time constant [microsec] for probability decay of FOE position
    private float tProb = getFloat("tProb", 1000f);

    //  ______filtering incoming events
    // valid flow vector angle in degrees, +- vector to center from current pixel
    private final double validAngle = 55 * Math.PI / 180.0;
    // turn diverging field around center filter on/off
    private boolean centralFilter = getBoolean("centralFilter", true);
    // center coordinates of current sensor
    private int centerX, centerY;
    // radius of evaluated pixels around current foe for probability update
    private int updateR = getInt("updateR", 50);
    
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
        flowFilter = new LucasKanadeFlow(chip); // new LocalPlanesFlow(chip);//TODO: make adaptable to other optical flow algos
        flowFilter.setDisplayRawInput(false); // to pass flow events not raw in        
        setEnclosedFilter(flowFilter);

        resetFilter();

        setPropertyTooltip("importGTfromFile", "imports text file to load ground truth FOE data");
        setPropertyTooltip("resetGroundTruth", "Resets the ground truth focus of expansion loaded from file.");
        setPropertyTooltip("Focus of Expansion", "tProb", "time constant [microsec]"
                + " for probability decay of FOE position");
        setPropertyTooltip("Focus of Expansion", "updateR", "radius [pixels] around current FOE where probability is updated");
        setPropertyTooltip("Focus of Expansion", "displayFOE", "shows the estimated focus of expansion (FOE)");
        setPropertyTooltip("Focus of Expansion", "centralFilter", "turn filter for diverging field around center on/off");
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

            if (centralFilter && getRelAngle(x - centerX, y - centerY, vx, vy) > validAngle) {
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
        //int currentGTIndex = binarySearch(ts*1e-6);
        //System.out.println("speed = " + filteredOutSpeed + " central = " + filteredOutCentral + " foeX = " + foeX + " foeY = " + foeY
        //+ " corresponding gt foeX=" + gtFoeX.get(currentGTIndex) + " foeY=" +gtFoeY.get(currentGTIndex) + " at time ts=" + ts
        //+ " with index " + currentGTIndex);
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
        foeX = centerX;
        foeY = centerY;
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
        log.info("import of ground truth done, data with times from "+gtTime.firstElement()+" to "+gtTime.lastElement());             
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
    
    public float getTProb() {
        return this.tProb;
    }

    public void setTProb(final float tProb_) {
        this.tProb = tProb_;
        putFloat("tProb", tProb_);
    }
    
    public int getUpdateR() {
        return this.updateR;
    }

    public void setUpdateR(final int updateR_) {
        this.updateR = updateR_;
        putInt("updateR", updateR_);
    }

    public boolean isDisplayFOE() {
        return displayFOE;
    }

    public void setDisplayFOE(boolean displayFOE_) {
        this.displayFOE = displayFOE_;
        putBoolean("displayFOE", displayFOE_);
    }
    
    public boolean isCentralFilter() {
        return centralFilter;
    }

    public void setCentralFilter(boolean centralFilter_) {
        this.centralFilter = centralFilter_;
        putBoolean("centralFilter", centralFilter_);
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
            int currentGTIndex = binarySearch(ts*1e-6);
            DrawGL.drawBox(gl, gtFoeX.get(currentGTIndex), gtFoeY.get(currentGTIndex), 4, 4, 0);
            gl.glPopMatrix();
        }
    }
    
    // returns the index of the timestamp in GT data closest to current time
    private int binarySearch(double time) {
        if (!importedGTfromFile) {
            return 0;
        }
        
        if(time < gtTime.firstElement()) {
            return 0;
        }
        if(time > gtTime.lastElement()) {
            return gtTime.size()-1;
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
