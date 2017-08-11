package eu.seebetter.ini.chips.davis;

import java.awt.Point;

import eu.seebetter.ini.chips.DavisChip;
import net.sf.jaer.Description;
import net.sf.jaer.DevelopmentStatus;
import net.sf.jaer.graphics.AEFrameChipRenderer;
import net.sf.jaer.hardwareinterface.HardwareInterface;

@Description("DAVIS APS-DVS camera with 320x240 pixels")
@DevelopmentStatus(DevelopmentStatus.Status.Experimental)
public class Davis320 extends DavisBaseCamera {

	public static final short WIDTH_PIXELS = 320;
	public static final short HEIGHT_PIXELS = 240;

	public Davis320() {
		setName("Davis320");
		setDefaultPreferencesFile("biasgenSettings/Davis640/DAVIS640_TestExp.xml");
		setSizeX(Davis320.WIDTH_PIXELS);
		setSizeY(Davis320.HEIGHT_PIXELS);

		setBiasgen(davisConfig = new DavisTowerBaseConfig(this));

		davisRenderer = new AEFrameChipRenderer(this);
		davisRenderer.setMaxADC(DavisChip.MAX_ADC);
		setRenderer(davisRenderer);

		setApsFirstPixelReadOut(new Point(getSizeX() - 1, getSizeY() - 1));
		setApsLastPixelReadOut(new Point(0, 0));
	}

	public Davis320(final HardwareInterface hardwareInterface) {
		this();
		setHardwareInterface(hardwareInterface);
	}
}
