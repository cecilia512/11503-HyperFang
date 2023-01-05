package org.firstinspires.ftc.robotcontroller.internal;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class visionTestHue {
    private LinearOpMode opMode;

    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK; // This is the webcam.
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AcELeNr/////AAABmeg7NUNcDkPigDGNImdu5slLREdKn/q+qfajHBypycR0JUZYbfU0q2yZeSud79LJ2DS9uhr7Gu0xDM0DQZ36GRQDgMRwB8lf9TGZFQcoHq4kVAjAoEByEorXCzQ54ITCextAucpL2njKT/1IJxgREr6/axNEL2evyKSpOKoNOISKR6tkP6H3Ygd+FHm2tF/rsUCJHN5bTXrbRbwt5t65O7oJ6Wm8Foz1npbFI0bsD60cug4CpC/Ovovt2usxIRG8cpoQX49eA2jPRRLGXN8y1Nhh9Flr0poOkYoCExWo2iVunAGOwuCdB/rp/+2rkLBfWPvzQzrN9yBBP0JVJZ4biNQ41qqiuVvlc31O9xEvbKHt";

    private final int RED_LOW = 90; //i guess we can steal this for orange detection
    private final int GREEN_LOW = 131; //hello future me here its actually like vomit green now
    private final int BLUE_LOW = 95; //red of 125, green 225, blue is 0
    private final int RED_HIGH = 107;
    private final int GREEN_HIGH = 151;
    private final int BLUE_HIGH = 115;

    private final int CYAN_RED_LOW = 30;
    private final int CYAN_RED_HIGH = 50;
    private final int CYAN_BlUE_LOW = 120; //no need for cyan blue high, because the high is 255
    private final int CYAN_BlUE_HIGH = 146;
    private final int CYAN_GREEN_LOW = 80; //same reasoning
    private final int CYAN_GREEN_HIGH = 110; //same reasoning

    private final int MAGENTA_RED_LOW = 113;
    private final int MAGENTA_RED_HIGH = 150;
    private final int MAGENTA_BLUE_LOW = 55; //i am assuming the same holds for magenta
    private final int MAGENTA_BLUE_HIGH = 85; //i am assuming the same holds for magenta
    private final int MAGENTA_GREEN_LOW = 20;
    private final int MAGENTA_GREEN_HIGH = 46;


    public visionTestHue(LinearOpMode opMode, char side) {

        this.opMode = opMode;

        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        params.vuforiaLicenseKey = VUFORIA_KEY;
        if (side == 'r') {
            params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam");
        } else {
            //whatever
        }


        vuforia = ClassFactory.getInstance().createVuforia(params);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Format returns 2 bytes per pixel in GGGBBBBB RRRRRGGG format (little-endian)
        vuforia.setFrameQueueCapacity(4);
        vuforia.enableConvertFrameToBitmap();

    }

    public int levelIdent(char side) {
        Bitmap bitmap = null;
        try {
            bitmap = getBitmap();
        } catch (InterruptedException e) {
            opMode.telemetry.addData("ERROR: ", "Cringemac ate lao gan ma and then aggressively started pooping his pants");
            opMode.telemetry.update();
        }

        int level = 3;

        int xVal; //x
        int yVal; //y
        int xIt; //x-val iterator
        int yIt = 1; //y-val iterator
        xVal = bitmap.getWidth() - 1;
        yVal = bitmap.getHeight() - 1;
        int mCount = 0;
        int gCount = 0;
        int cCount = 0;


        for (yIt = 1; yIt < bitmap.getHeight(); yIt += 2) {
            yVal = bitmap.getHeight() - yIt;
            xIt = 1;
            xVal = bitmap.getWidth() - xIt;
            while (xIt < 550) {
                xVal = bitmap.getWidth() - xIt;
                if (isMagenta(bitmap.getPixel(xVal, yVal))) {
                    mCount++;
                }
                if (isCyan(bitmap.getPixel(xVal, yVal))) {
                    cCount++;
                }
                if (isOrange(bitmap.getPixel(xVal, yVal))) {
                    gCount++;
                }
                xIt += 3;
            }
            if (yIt >= 470) {
                level = 0;
            }
        }

        if (mCount > gCount) {
            if (mCount > cCount) {
                level = 2;
            } else {
                level = 1;
            }
        } else {
            if (cCount > gCount) {
                level = 1;
            } else {
                level = 3;
            }
        }

        return level;
    }

    private boolean isCyan(int pixel) {
        return ((CYAN_RED_LOW <= red(pixel)) && (red(pixel) <= CYAN_RED_HIGH) && (CYAN_GREEN_LOW <= green(pixel)) &&
                (green(pixel) <= CYAN_GREEN_HIGH) && (CYAN_BlUE_LOW <= blue(pixel)) && (blue(pixel) <= CYAN_BlUE_HIGH));
    }

    private boolean isOrange(int pixel) {
        return ((RED_LOW <= red(pixel)) && (red(pixel) <= RED_HIGH) && (GREEN_LOW <= green(pixel)) &&
                (green(pixel) <= GREEN_HIGH) && (BLUE_LOW <= blue(pixel)) && (blue(pixel) <= BLUE_HIGH));
    }


    public boolean isMagenta(int pixel) {
        return ((MAGENTA_RED_LOW <= red(pixel)) && (red(pixel) <= MAGENTA_RED_HIGH) && (MAGENTA_GREEN_LOW <= green(pixel)) &&
                (green(pixel) <= MAGENTA_GREEN_HIGH) && (MAGENTA_BLUE_LOW <= blue(pixel)) && (blue(pixel) <= MAGENTA_BLUE_HIGH));
    }

    public Bitmap getBitmap() throws InterruptedException {

        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        for (int i = 0; i < numImages; i++) {

            int format = picture.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;
            }
        }

        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

        picture.close();

        return imageBitmap;
    }

    public void toHSV(double r, double b, double g){

        double hmax;
        double hmin;
        double hdif;
        double h = -1, s = -1, v = -1;

        r = r / 255.0;
        b = b / 255.0;
        g = g / 255.0;

        hmax = Math.max(r, Math.max(g,b));
        hmin = Math.min(r, Math.min(g,b));
        hdif = hmax - hmin;

        if (hdif == 0) h = 0;
        else if (hmax == r ) h =      (g - b)   / hdif; //might change to 60(g - b) /hdif?
        else if (hmax == g ) h = (2 + (b - r) ) / hdif;
        else if (hmax == b ) h = (4 + (r - g) ) / hdif;

        if (hmax == 0) s = 0;

        v = hmax;
    }
}

