/**
 * @file
 * 
 * ESP32 AWS Library
 * 
 * Functions to get the crawler coordinates from the Camera over AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */

#ifndef aws_h
#define aws_h

#include <MQTTClient.h>

/* The MQTT topics that this device should publish/subscribe to */
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC_ROVER  "esp32/rover"
#define AWS_IOT_SUBSCRIBE_TOPIC_TARGET "esp32/target"

void messageHandler(String, String);
class myawsclass {
  public:
    myawsclass();

    boolean connectionEstablishedAWS = false;
    String subscriptionTopic = "";
    String subscriptionPayload = "";
    boolean msgRcvIndicator = false;

    void connectAWS();                            /* Initialize and connect to AWS */
    void publishMessage(int16_t sensorValue);     /* Publish the values of the sensors */
    void stayConnected();                         /* Maintain the connection */
    String getSubscriptionTopic (void);
    String getSubscriptionPayload (void);
};

extern myawsclass awsObject;

#endif