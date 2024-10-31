#include "model.h"
#include <ArduinoCloud.h>  


const char DEVICE_KEY[] = "device_key";
const char DEVICE_ID[] = "device_id";
const char NETWORK_SSID[] = "ssid";
const char NETWORK_PASSWORD[] = "password";

double prediction; 


CloudDouble cloudPrediction;
MLP mlp(NET_INPUTS, NET_OUTPUTS, layerSizes, MLP::LOGISTIC, initW, true);

void initArduinoCloud() {
  ArduinoCloud.addProperty(cloudPrediction, READWRITE, ON_CHANGE, updateCloud);
  
  
  ArduinoCloud.begin(DEVICE_ID, DEVICE_KEY);
  WiFi.begin(NETWORK_SSID, NETWORK_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}


void forwardPass(double input[]) {

  double netOutput[] = { 0.7, 0.3 }; 
  

  mlpClass = classify(netOutput);


  if (strcmp(mlpClass, "YES") == 0) {
    prediction = 1.0;
  } else {
    prediction = 0.0;
  }
  

  cloudPrediction = prediction;
}

char* classify(double output[]) {

  if (output[0] > output[1]) {
    return Class[0];  // YES
  } else {
    return Class[1];  // NO
  }
}


void updateCloud() {
  Serial.print("Updated cloud value: ");
  Serial.println(cloudPrediction);
}

void setup() {
  Serial.begin(115200);
  initArduinoCloud();


}

void loop() {
  ArduinoCloud.update();
  
  netInput[1] = Yes;
  netInput[2] = No;
  

  
  int index = mlp.getActivation(netInput);
  mlpClass = Class[index];
  
 
  forwardPass(netInput);
  
  delay(10000);  
}
