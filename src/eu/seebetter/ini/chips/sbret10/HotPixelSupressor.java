/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package eu.seebetter.ini.chips.sbret10;

import java.awt.Point;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;
import javax.media.opengl.GL;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLException;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.chip.AEChip;
import net.sf.jaer.event.ApsDvsEvent;
import net.sf.jaer.event.BasicEvent;
import net.sf.jaer.event.EventPacket;
import net.sf.jaer.event.OutputEventIterator;
import net.sf.jaer.eventprocessing.EventFilter2D;
import net.sf.jaer.eventprocessing.FilterChain;
import net.sf.jaer.eventprocessing.filter.EventRateEstimator;
import net.sf.jaer.graphics.FrameAnnotater;

/**
 * Cheaply suppresses hot pixels from DVS; ie pixels that continuously fire a
 * sustained stream of events.
 * <p>
 * The method is based on a list of hot pixels that is filled probabalistically.
 * A hot pixel is likely to occupy the list and addresses in the list are not
 * output. The list is filled at with some baseline probability which is reduced
 * inversely with the average event rate. This way, activity induced output of
 * the sensor does not overwrite the stored hot pixels.
 *
 * @author tobi
 */
@Description("Cheaply suppresses hot pixels from DVS; ie pixels that continuously fire events when when the visual input is idle.")
@DevelopmentStatus(DevelopmentStatus.Status.Experimental)
public class HotPixelSupressor extends EventFilter2D implements FrameAnnotater{

    private int numHotPixels = getInt("numHotPixels", 30);
    private float baselineProbability = getFloat("baselineProbability", 1e-3f);
    private float thresholdEventRate=getFloat("thresholdEventRate",10e3f);
    private HotPixelSet hotPixelSet = new HotPixelSet();
    private boolean showHotPixels=getBoolean("showHotPixels", true);
    
    private Random r = new Random();
    private EventRateEstimator eventRateEstimator;
    
    private class HotPixel{
        int x,y,lasttimestamp,address;

        HotPixel(BasicEvent e){
            address=e.address;
            x=e.x; y=e.y; lasttimestamp=e.timestamp;
        }
        int timeSinceLast(BasicEvent e){
            return e.timestamp-lasttimestamp;
        }
        void updateLastTimestamp(BasicEvent e){
            lasttimestamp=e.timestamp;
        }

        @Override
        public boolean equals(Object obj) {
           if(obj instanceof BasicEvent){
               return ((BasicEvent)obj).address==address;
           }else if(obj instanceof HotPixel){
               return ((HotPixel)obj).address==address;
           }else return false;
        }
        
        public String toString(){
            return String.format("(%d,%d)",x,y);
        }

        public int hashCode() {
            return new Integer(address).hashCode();
        }
    }
    
    private class HotPixelSet extends HashSet<HotPixel>{
        boolean contains(BasicEvent e){
            return contains(new HotPixel(e));
        }
    }

    public HotPixelSupressor(AEChip chip) {
        super(chip);
        setPropertyTooltip("numHotPixels", "maximum number of hot pixels");
        setPropertyTooltip("baselineProbability","probability that an address is written to the hot pixel set");
        setPropertyTooltip("thresholdEventRate","pixels are only stored to the hot pixel list when the event rate is below this threshold, \n so that normal activity of the sensor does not overwrite the hot pixels");
        eventRateEstimator=new EventRateEstimator(chip);
        setEnclosedFilterChain(new FilterChain(chip));
        eventRateEstimator.setEnclosed(true, this);
        getEnclosedFilterChain().add(eventRateEstimator);
    }

    @Override
    synchronized public EventPacket<?> filterPacket(EventPacket<?> in) {
        checkOutputPacketEventType(in);
        OutputEventIterator outItr=out.outputIterator();
        eventRateEstimator.filterPacket(in);
        float rate=eventRateEstimator.getFilteredEventRate();
        for (BasicEvent e : in) {
            if (rate<thresholdEventRate) {
                
               // find oldest hot pixel and replace
                int oldestTimestamp=Integer.MAX_VALUE;
                HotPixel oldest=null;
                for (HotPixel i : hotPixelSet) {
                    if(i.lasttimestamp<oldestTimestamp){
                        oldestTimestamp=i.lasttimestamp;
                        oldest=i;
                    }
                }
                if(hotPixelSet.size()>=numHotPixels){
                     hotPixelSet.remove(oldest);
                }
                hotPixelSet.add(new HotPixel(e));
            }
            if (rate>=thresholdEventRate&& hotPixelSet.contains(e)) {
                continue;
            }
            ApsDvsEvent a = (ApsDvsEvent) outItr.nextOutput();
            a.copyFrom(e);

        }
        return out;
    }

    @Override
    synchronized public void resetFilter() {
        hotPixelSet.clear();
        
    }

    @Override
    public void initFilter() {
    }

    /**
     * @return the numHotPixels
     */
    public int getNumHotPixels() {
        return numHotPixels;
    }

    /**
     * @param numHotPixels the numHotPixels to set
     */
    public void setNumHotPixels(int numHotPixels) {
        this.numHotPixels = numHotPixels;
        putInt("numHotPixels",numHotPixels);
    }

    /**
     * @return the baselineProbability
     */
    public float getBaselineProbability() {
        return baselineProbability;
    }

    /**
     * @param baselineProbability the baselineProbability to set
     */
    public void setBaselineProbability(float baselineProbability) {
        this.baselineProbability = baselineProbability;
        putFloat("baselineProbability",baselineProbability);
    }

    /**
     * @return the thresholdEventRate
     */
    public float getThresholdEventRate() {
        return thresholdEventRate;
    }

    /**
     * @param thresholdEventRate the thresholdEventRate to set
     */
    public void setThresholdEventRate(float thresholdEventRate) {
        this.thresholdEventRate = thresholdEventRate;
        putFloat("thresholdEventRate",thresholdEventRate);
    }

    @Override
    public void annotate(GLAutoDrawable drawable) {
        if(!showHotPixels) return;
        GL gl = drawable.getGL();
           try{
                gl.glEnable(GL.GL_BLEND);
                gl.glBlendFunc(GL.GL_SRC_ALPHA,GL.GL_ONE_MINUS_SRC_ALPHA);
                gl.glBlendEquation(GL.GL_FUNC_ADD);
            }catch(GLException e){
                e.printStackTrace();
                showHotPixels=false;
            }
        gl.glColor4f(.5f, .5f, .5f,.5f);
        gl.glLineWidth(1f);
        for (HotPixel p : hotPixelSet) {
            gl.glRectf(p.x - 1, p.y - 1, p.x + 2, p.y + 2);
        }
    }

    /**
     * @return the showHotPixels
     */
    public boolean isShowHotPixels() {
        return showHotPixels;
    }

    /**
     * @param showHotPixels the showHotPixels to set
     */
    public void setShowHotPixels(boolean showHotPixels) {
        this.showHotPixels = showHotPixels;
        putBoolean("showHotPixels", showHotPixels);
    }
}
