/**
 * ESP32 secrets file
 * 
 * Contains information required to connect to WiFi and in-turn, AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */


/*
  Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <pgmspace.h>

#define SECRET
#define THINGNAME "Rover"

const char WIFI_SSID[] = "Galaxy S10 Lite";
const char WIFI_PASSWORD[] = "Vishal@2";
//const char WIFI_SSID[] = "Vodafone-13B3";
//const char WIFI_PASSWORD[] = "AAULCTDcbCqTABGa";
//const char WIFI_SSID[] = "UPCBD8B266";
//const char WIFI_PASSWORD[] = "zM2wkceNrvmk";
const char AWS_IOT_ENDPOINT[] = "acvnitw3zyx9d-ats.iot.us-east-2.amazonaws.com";

/* Amazon Root CA 1 */
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

/* Device Certificate */
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUHw0WopswUKyS/bvMoZg7hSngWIMwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIxMTIxOTE0MTEy
NloXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKNlxaCyqIkfAxWqwH+7
+vM2E95FLKH7Fbo/VpWSLE3qvYNzEB/eTI7Foo0INxbEOvLMo8WNYtwJ7aSdru7d
f6lCtGoQEUrzZ7iZ3SzwpJfd0pTm6Y3YcXSeOMJ6c+y4pHgNG99EKEIyDac34V/S
LguFIeY6BkwEnUo5m1MNv0ceb4S/44ZurjaKOYWAuktp/J9ftr0fpuYw7hx+weU3
k9jQlC/9y8pitvpJqHFMu/TLdDcWroHi4/ZD6HAC0F0B0EIN0pnoUQN8zbKOMwZ+
PgEhFUOEXEEd/nirmevqWmwN1UKV8BJP+IA63RmMEV1xSs4X/BTXKROXUgnWpjB8
ZP8CAwEAAaNgMF4wHwYDVR0jBBgwFoAUKugex/C+y33TO9r2Ur5TNniqo1cwHQYD
VR0OBBYEFOeo8JFycG1J77K5KY9iEu4bx54KMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQA9ptOfVmKZx6qNj0scX6k54PhE
fdnDUEt5x++Cbc+Vsui/9TzOr/uE2VzqFpYNbzQurHqvMxS8N7ggAD1EWXD2qbFL
twkIoUBU1avYW2saTlgNU2i8VOPQqT/5b42PE2rIou+M5c2qcGSWMGFwIDW4X+IA
e9lckiyhcG71V7B3d71z2BoIW7emk3QdwYF/kuL8nCU8Wj3tPOjEgD6yTORfI1+e
EhL5BFTJmlrzNivcmPmgbh3NII7FSaPiYlCnnRV8Yu2LQKHrIHQmQh+f9aqgyNfd
XlLcj0qWUI7mSQ+K9e4b4mmpIjlDXhv85YmreNgB6Idsq0Vb1kC5eVPGIQNm
-----END CERTIFICATE-----
)KEY";

/* Device Private Key */
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEAo2XFoLKoiR8DFarAf7v68zYT3kUsofsVuj9WlZIsTeq9g3MQ
H95MjsWijQg3FsQ68syjxY1i3AntpJ2u7t1/qUK0ahARSvNnuJndLPCkl93SlObp
jdhxdJ44wnpz7LikeA0b30QoQjINpzfhX9IuC4Uh5joGTASdSjmbUw2/Rx5vhL/j
hm6uNoo5hYC6S2n8n1+2vR+m5jDuHH7B5TeT2NCUL/3LymK2+kmocUy79Mt0Nxau
geLj9kPocALQXQHQQg3SmehRA3zNso4zBn4+ASEVQ4RcQR3+eKuZ6+pabA3VQpXw
Ek/4gDrdGYwRXXFKzhf8FNcpE5dSCdamMHxk/wIDAQABAoIBAG8R7j4nU+Mp3onq
8UGjAZoz51uPACD3sbRR+8RegZlReROSsqJHFRl0BOQDcpcaOsvzCVxvmITkhtMA
kPo0PORMwKDcXTTqslXM2YkD546ILe4j7R0LxUu1hfXFEWoXtU0Z9BlXllow9hD2
LlBZag1QqVN7m+4usNDUyd2yGZT4aEV49aOht6iDMreni/FWhr82frm0E0qEsFpj
GiHOxTQY8bZlEgvCrQKf5c0WooYPPM8sVBjI6A0BiCtw8Ni99x7r53UvRHESm4wH
q8/fMXDSgkYntkL2FZabeP84XvaCeR/q2vwYLGhwX77jiDLYIAtvqfxUvic+nl1h
OypNZKECgYEA0fP3HZJ47IQCc8K37VhZYJRnFxz1oPpv0I9tT2gdDiA3PvlZrChC
jXAZTcyxcQLxVdlYCGdz9/9qqWRcg0CytOg1zSGOju1zE3WmyDZ+UWPcjfxmrPHS
GcqRY2V5HfxKQFsDMoMARTBZ/3n8w4drONc6Km2xPLFD7PxeL63P7nECgYEAxzvm
06M/pKoy49DZqb9H2B+yhcbeK2tGTuDfSfiD/kKSNVR73HayAOearcQT3KYF2CdA
AxeYMaQOQyYN1vQIcuqe2+NYjy0R0hrpi1oJMQmfpltQnoU5pppFXB7TTfJhBp6U
3gKuuiQdS25olXDWenvdXkld+7c8sVBuTka5Im8CgYBIUvJdjYd6cj8mAX26lSS0
0Gpy29cBjUPXZJETOZpIs3BUkTECzRuf/W26DzlQ5OKd4DEjNAAO/j2P/LKhxDfL
efKOp3Cka8eofG7nqV36S5w+jSQRj3wAxncu9lWyrqYlZTTEwbvmGbHm/+7CrHOT
MREobIERlz7+9DC92/I1AQKBgCLqJ3h9SmD0BrkP16U8xT1lyKZTBIo9YXPfSqIu
8Aq15eN8q7wl/OnVrLC5BPYNDkckEb64+s3wPyObXp+F4pko7rAANnvY3R+h4b2v
CJ9UXWsM46L0G/dLX85WBJfLWx+K2PHTLIWoYI3gimlt/dg3LsMgPl0nbnZwbvQ+
xqD3AoGAR6K1YXLG7sXt2qHYqQ2w4Ov9mb9QHwl7EVTm5m86vyXxOl7ZlhrOAhXU
0lanwzL8+0HesmorZ6icBDvMdDDo80Yobmyd/ynseJCsp0Wyz0lbbyeHAp0AkpeC
9RrzuqyBXW9opVGDmKOFITPgadY1ByAL9zPS0bvWQA+kf62eXdI=
-----END RSA PRIVATE KEY-----
)KEY";