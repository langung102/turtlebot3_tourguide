/*
 * turtlebot3Firebase.h
 *
 *  Created on: Feb 20, 2024
 *      Author: Dinhthong
 */

#ifndef INC_TURTLEBOT3FIREBASE_H_
#define INC_TURTLEBOT3FIREBASE_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <assert.h>
#include <thread>
#include <firebase/app.h>
#include <firebase/database.h>
#include "firebase/future.h"
#include "firebase/util.h"
#include <cstdlib> 
#include <vector>
#include <sstream>
/*Database Information*/
extern firebase::App* firebase_app;
extern const char *databasePath;
extern const char *requestPath;
extern const char *email;
extern const char *password;
extern const char *api_key;
extern const char *app_id;
extern const char *database_url;
extern const char *storage_bucket;
extern const char *project_id;

/*Initialize Firebase*/
void InitializeFirebase();

extern firebase::AppOptions options;
extern int numberOfStation;
/*Battery information*/
void SetBattery(int value);
int GetBattery();

void isReachStation(int value);

/*wait for getting and setting successfully*/
void WaitForCompletion(const firebase::FutureBase &future, const char *name);

struct stationData{
    std::string description;
    int id;
    std::string nameStation;
    std::string multipleStation;
};

/*get request information*/
struct getRequestData{
    int id;
    double yPosition;
    double xPosition;
    double yaw;
    stationData station;
};

getRequestData getRequest();

struct getPositionData{
    double xPosition;
    double yPosition;
    double yaw;
};
struct multiStation{
    std::string name;
    double xPosition;
    double yPosition;
    double yaw;
};
//struct save data for multiple Station
struct Zone {
    std::string name;
    double value1;
    double value2;
    int value3;
};
extern std::vector<Zone> zones;
stationData getMultiStation();
void parseMultiStation(std::string str);
getPositionData getPosition();

void setPosition(double x, double y, double yaw);

void setStatus(bool value);

stationData getStation();
#endif /* INC_TURTLEBOT3FIREBASE_H_ */
