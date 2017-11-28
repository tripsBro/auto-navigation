////#include <Arduino.h>
////
////void distance(){
////    float flat1=12.9697;     // flat1 = our current latitude. flat is from the gps data.
////    float flon1=79.1572;  // flon1 = our current longitude. flon is from the fps data.
////    float dist_calc=0;
////    float dist_calc2=0;
////    float diflat=0;
////    float diflon=0;
////    float x2lat= 12.97060  ;  //enter a latitude point here   this is going to be your waypoint
////    float x2lon= 79.1595;
//////---------------------------------- distance formula below. Calculates distance from current location to waypoint
////    diflat=radians(x2lat-flat1);  //notice it must be done in radians
////    flat1=radians(flat1);    //convert current latitude to radians
////    x2lat=radians(x2lat);  //convert waypoint latitude to radians
////    diflon=radians((x2lon)-(flon1));   //subtract and convert longitudes to radians
////    dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
////    dist_calc2= cos(flat1);
////    dist_calc2*=cos(x2lat);
////    dist_calc2*=sin(diflon/2.0);
////    dist_calc2*=sin(diflon/2.0);
////    dist_calc +=dist_calc2;
////    dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
////    dist_calc*=6371000.0; //Converting to meters
////    Serial.println("distance");
////    Serial.println(dist_calc);    //print the distance in meters
////}
////
////void head() {
////    float x2lat = 12.97060;
////    float flat1 = 12.9697;
////
////    float flon1 = radians(79.1572);  //also must be done in radians
////
////    float x2lon = radians(79.1595);  //radians duh.
////
////    float heading = atan2(sin(x2lon - flon1) * cos(x2lat), cos(flat1) * sin(x2lat) - sin(flat1) * cos(x2lat) * cos(x2lon - flon1));
////
////    heading = heading * 180 / 3.1415926535;  // convert from radians to degrees
////
////    int head = heading; //make it a integer now
////
////    if (head < 0) {
////
////        heading += 360;   //if the heading is negative then add 360 to make it positive
////
////    }
////
////    Serial.println("heading:");
////
////    Serial.println(heading);   // print the heading.
////
////}
////
////int heading;
////void Turn(int currentHeading)
////{
////    int delta = currentHeading-heading;
////    int turn;
////    if(delta>=-180){
////
////        if(delta<=0){
////
////            turn=6;    //set turn =6 which means "right"
////
////        }
////
////    }
////
////    if(delta<-180){
////
////        turn=4;      //set turn = 4 which means "left"
////
////    }
////
////    if(delta>=0){
////
////        if(delta<180){
////
////            turn=4;   //set turn = 4 which means "left"
////
////        }
////
////    }
////
////    if(delta>=180){     //set turn =6 which means "right"
////
////        turn=6;
////
////    }
////    if(delta==0){
////
////        turn=0;   //then set turn = 0 meaning go "straight"
////
////    }
////
////
////
////}
////
////void setup()
////{
////    Serial.begin(9600);
////
////}
////
////void loop()
////{
////    distance();
////    head();
////    delay(1000);
////
////}
// // my method
//#include <SoftwareSerial.h>
//
//#include <TinyGPS.h>
///* This sample code demonstrates the normal use of a TinyGPS object.
//   It requires the use of SoftwareSerial, and assumes that you have a
//   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
//*/
//
//TinyGPS gps;
//SoftwareSerial ss(4, 3);
//static void smartdelay(unsigned long ms);
//
//void setup()
//{
//    Serial.begin(115200);
//
//    ss.begin(4800);
//}
//void loop()
//{
//    float flat, flon;
//    unsigned long age, date, time, chars = 0;
//    unsigned short sentences = 0, failed = 0;
//    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
//    gps.f_get_position(&flat, &flon, &age);
////  Serial.print("lat: ");
////  Serial.println(flat);
////  Serial.print("lon: ");
////  Serial.println(flon);
//
//    if (flat == TinyGPS::GPS_INVALID_F_ANGLE)
//    {
//        Serial.println("*invalid*");
//    }
//    else
//    {
//        Serial.print(flat, 6);
//        int vi = abs((int)flat);
//        int flen = 6 + (flat < 0.0 ? 2 : 1); // . and -
//        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
//        for (int i=flen; i<10; ++i)
//            Serial.print(' ');
//    }
//    if (flon == TinyGPS::GPS_INVALID_F_ANGLE)
//    {
//        Serial.println("*invalid lon*");
//    }
//    else
//    {
//        Serial.print(flon, 6);
//        int vi = abs((int)flon);
//        int flen = 6 + (flon < 0.0 ? 2 : 1); // . and -
//        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
//        for (int i=flen; i<10; ++i)
//            Serial.print(' ');
//    }
//
//    Serial.println();
//
//    smartdelay(1000);
//}
//
//static void smartdelay(unsigned long ms)
//{
//    unsigned long start = millis();
//    do
//    {
//        while (ss.available())
//            gps.encode(ss.read());
//    } while (millis() - start < ms);
//}
