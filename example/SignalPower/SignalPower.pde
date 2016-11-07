import processing.serial.*;
import org.gicentre.utils.stat.*;
import processing.core.PVector;
import java.util.List;

XYChart lineChart;
XYChart lineChart2;
Serial myPort;

int Measure1 = 0;
int Measure2 = 0;
int Signal1 = 0;
int Signal2 = 0;
int counter = 0;
boolean newValue = false;
List<PVector> vectors;
float measures1[] = new float[500];
float measures2[] = new float[500];
float signal1[] = new float[500];
float signal2[] = new float[500];
float counterVecor[] = new float[500];

void setup()
{
  size(800,400);
  frameRate(100);
  smooth();
 
  myPort = new Serial(this, Serial.list()[1], 57600);
  delay(100);
  myPort.bufferUntil('\n');
  myPort.bufferUntil('\n');
  myPort.bufferUntil('\n');
  
  textFont(createFont("Arial",15),15); 
  lineChart = new XYChart(this);
  lineChart.showXAxis(true); 
  lineChart.showYAxis(true); 
  lineChart.setMinY(0);
  lineChart.setMaxY(255);
  lineChart.setYFormat("##0");
  lineChart.setXFormat("#0.000");   
  lineChart.setLineColour(color(200,125,125,255));
  lineChart.setPointSize(0);
  lineChart.setLineWidth(2);
  
  lineChart2 = new XYChart(this);
  lineChart2.showXAxis(true); 
  lineChart2.showYAxis(true); 
  lineChart2.setMinY(0);
  lineChart2.setMaxY(255);
  lineChart2.setYFormat("##0");
  lineChart2.setXFormat("#0.000");   
  lineChart2.setLineColour(color(0,128,255,255));
  lineChart2.setPointSize(0);
  lineChart2.setLineWidth(2);
  for(int i = 0; i<500; i++){
    counterVecor[i] = i;
  }
}

void draw()
{
  background(255);
  textSize(9);
  lineChart.draw(15,15,800-30,400-30);
  lineChart2.draw(15,15,800-30,400-30);
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String in = myPort.readStringUntil('\n');
  if (in != null) {
    String splitted[] = in.split("\t");
      if(splitted.length > 7){
      Measure1 = int(splitted[1]);
      Signal1 = int(splitted[3]);
      Measure2 = int(splitted[5]);
      Signal2 = int(splitted[7]);
      }
    addData(Measure1, Measure2, Signal1, Signal2);
  }
}


void addData(int ma, int mb, int sc, int sd){
  measures1[counter] = ma;
  measures2[counter] = mb;
  signal1[counter] = sc;
  signal2[counter] = sd;
  counter += 1;
  counter = counter%500;
  lineChart.setData(counterVecor, signal1);
  lineChart2.setData(counterVecor, signal2);
}