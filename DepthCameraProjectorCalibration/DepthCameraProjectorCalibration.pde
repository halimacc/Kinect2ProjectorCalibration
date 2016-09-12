//==========================================================
// set resolution of your projector image/second monitor,
// intrinsics of your depth camera,
// and name of your calibration file-to-be

// projector size
int projectorScreenId = 2;
int pWidth = 1024;
int pHeight = 768;

//camera information based on the Kinect v2 hardware
// you can get it from libfreenect2 program or calibrate it yourself
static class DepthCameraParams {
  static float cx = 263.088196f;
  static float cy = 207.137207f;
  static float fx = 365.753296f;
  static float fy = 365.753296f;
}

// calibration file-to-be
String calibFilename = "calibration.txt";


//==========================================================
//==========================================================

int depthWidth = 512;
int depthHeight = 424;

int colorWidth = 1920;
int colorHeight = 1080;

import KinectPV2.*;
import gab.opencv.*;
import controlP5.*;
import Jama.*;

KinectPV2 kinect;
OpenCV opencv;
ProjectorApplet pa;

int[] depthRaw;
PImage registeredImg;
int[] zeroPixels;

int frame;
int cacheLen = 1;
ArrayList<int[]> frameCache;
int[][] frameSum;
ArrayList<int[]> depthCache;
int[] depthSum;

ArrayList<PVector> foundPoints = new ArrayList<PVector>();
ArrayList<PVector> projPoints = new ArrayList<PVector>();
ArrayList<PVector> ptsK, ptsP;
PVector testPoint, testPointP;
boolean isSearchingBoard = false;
boolean calibrated = false;
boolean testingMode = false;
int cx, cy, cwidth;

void settings() {
  size(1200, 768);

  // start projector window
  pa = new ProjectorApplet();
  String[] args = {"ChessboardApplet"};
  PApplet.runSketch(args, pa);
}

void setup() 
{
  textFont(createFont("Courier", 24));
  
  opencv = new OpenCV(this, depthWidth, depthHeight);
 
  // initialize registered depth image
  registeredImg = createImage(depthWidth, depthHeight, PImage.RGB);
  
  frame = -1;
  frameSum = new int[depthWidth * depthHeight][3];
  depthSum = new int[depthWidth * depthHeight];
  depthRaw = new int[depthWidth * depthHeight];
  zeroPixels = new int[depthWidth * depthHeight];
  for (int i = 0; i < depthWidth * depthHeight; i++) {
    depthSum[i] = 0;
    zeroPixels[i] = 0;
    for (int j = 0; j < 3; ++j)
      frameSum[i][j] = 0;
  }
  
  frameCache = new ArrayList<int[]>();
  depthCache = new ArrayList<int[]>();
  for (int i = 0; i < cacheLen; ++i) {
    frameCache.add(new int[depthWidth * depthHeight]);
    depthCache.add(null);
  }

  // set up kinect
  kinect = new KinectPV2(this);
  kinect.enableDepthImg(true);
  kinect.enableColorImg(true);
  kinect.enablePointCloud(true);
  kinect.init();

  // matching pairs
  ptsK = new ArrayList<PVector>();
  ptsP = new ArrayList<PVector>();
  testPoint = new PVector();
  testPointP = new PVector();
  setupGui();
}

void draw() 
{ 
  frame++;
  
  int[] depthRawData = kinect.getRawDepthData();
  int[] dCache = depthCache.get(frame % cacheLen);
  for (int i = 0; i < depthWidth * depthHeight; ++i) {
    if (frame >= cacheLen) {
      depthSum[i] -= dCache[i];
    }
    depthSum[i] += depthRawData[i];
    depthRaw[i] = (int)(depthSum[i] / cacheLen);
  }
  depthCache.set(frame % cacheLen, depthRawData);
  
  mirrorPixels(depthRaw, depthWidth, depthHeight);
  
  // get mirror registered depth image
  int[] rPixels = new int[depthWidth * depthHeight];
  PApplet.arrayCopy(zeroPixels, rPixels);
  float[] mapDTC = kinect.getMapDepthToColor();
  PImage colorImg = kinect.getColorImage();
  // mapping color pixels to depth pixels
  colorImg.loadPixels();
  for (int y = 0; y < depthHeight; y++) {
    for (int x = 0; x < depthWidth; x++) {
      //incoming pixels 512 x 424 with position in 1920 x 1080
      int idx = y * depthWidth  + x;
      
      int  valXColor = (int)(mapDTC[2 * idx + 0]);
      int  valYColor = (int)(mapDTC[2 * idx + 1]);

      if (valXColor >= 0 && valXColor < colorWidth && valYColor >= 0 && valYColor < colorHeight) {
        rPixels[idx] = colorImg.pixels[valYColor * colorWidth + valXColor];
      }
    }
  }
  
  // update frame sum and registerd image
  int[] cache = frameCache.get(frame % cacheLen);
  registeredImg.loadPixels();
  for (int i = 0; i < depthWidth * depthHeight; ++i) {
    registeredImg.pixels[i] = 0;
    for (int j = 0; j < 3; ++j) {
      int mask = 0xff << (j * 8);
      if (frame >= cacheLen) {
        frameSum[i][j] -= (cache[i] & mask);
      }
      frameSum[i][j] += rPixels[i] & mask;
      registeredImg.pixels[i] += (int)(frameSum[i][j] / cacheLen ) & mask;
    }
    cache[i] = rPixels[i];
    registeredImg.pixels[i] |= 0xff000000;
  }
  
  mirrorPixels(registeredImg.pixels, depthWidth, depthHeight);
  registeredImg.updatePixels();

  opencv.loadImage(registeredImg);
  opencv.gray();

  if (isSearchingBoard)
    foundPoints = opencv.findChessboardCorners(4, 3);

  drawGui();
}

void drawGui() 
{
  background(0, 100, 0);

  // draw the RGB image
  pushMatrix();
  translate(30, 120);
  textSize(22);
  fill(255);
  image(registeredImg, 0, 0);
  
  // draw chessboard corners, if found
  if (isSearchingBoard) {
    int numFoundPoints = 0;
    for (PVector p : foundPoints) {
      if (getDepthMapAt((int)p.x, (int)p.y).z > 0) {
        fill(0, 255, 0);
        numFoundPoints += 1;
      } else  fill(255, 0, 0);
      ellipse(p.x, p.y, 5, 5);
    }
    if (numFoundPoints == 12)  guiAdd.show();
    else                       guiAdd.hide();
  } else  guiAdd.hide();
  if (calibrated && testingMode) {
    fill(255, 0, 0);
    ellipse(testPoint.x, testPoint.y, 10, 10);
  }
  popMatrix();

  // draw GUI
  pushMatrix();
  pushStyle();
  translate(depthWidth +70, 40);
  fill(0);
  rect(0, 0, 450, 680);
  fill(255);
  text(ptsP.size()+" pairs", 26, guiPos.y+525);
  popStyle();
  popMatrix();
}

void addPointPair() {
  if (projPoints.size() == foundPoints.size()) {
    for (int i=0; i<projPoints.size(); i++) {
      ptsP.add( projPoints.get(i));
      ptsK.add( getDepthMapAt((int) foundPoints.get(i).x, (int) foundPoints.get(i).y) );
    }
  }
  guiCalibrate.show();
  guiClear.show();
}

PVector getDepthMapAt(int x, int y) {
  int z = depthRaw[depthWidth * y + x];
  return covertDepthToCamera(x, y, (float)z / 1000.0f);
}

void clearPoints() {
  ptsP.clear();
  ptsK.clear();
  guiSave.hide();
}

void saveC() {
  String[] coeffs = getCalibrationString();
  saveStrings(dataPath(calibFilename), coeffs);
}

void loadC() {
  println("load");
  
  String[] s = loadStrings(dataPath(calibFilename));
  x = new Jama.Matrix(11, 1);
  for (int i=0; i<s.length; i++)
    x.set(i, 0, Float.parseFloat(s[i]));
  calibrated = true;
  
  println("done loading");
  guiTesting.addItem("Testing Mode", 1);
}

void mousePressed() {
  if (calibrated && testingMode) {
    testPoint = new PVector(constrain(mouseX - 30, 0, depthWidth -1), 
      constrain(mouseY - 120, 0, depthHeight - 1));

    testPointP = convertKinectToProjector(getDepthMapAt((int) testPoint.x, (int) testPoint.y));
  }
}

void mirrorPixels(int[] arr, int w, int h)
{
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w / 2; ++x) {
      int idx = y * w + x;
      int mirrorIdx = y * w + w - 1 - x;

      int tmp = arr[idx];
      arr[idx] = arr[mirrorIdx];
      arr[mirrorIdx] = tmp;
    }
  }
}