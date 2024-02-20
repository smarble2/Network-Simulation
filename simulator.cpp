#include "nodes.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>

int main(){

    //FRIST create a vector to hold the SensorNodes
    vector<SensorNode>mySensors;
    //rand coords for gateway
    time_t currentTime = time(0);
    string startTime = ctime(&currentTime);

    srand(static_cast<unsigned>(time(nullptr)));
    int gX = rand() % 1001; //for random coords for gateway
    int gY = rand() % 1001; //gateway
    int tX = rand() % 1001; //for random coords for gateway
    int tY = rand() % 1001; //gateway
    int goalX = rand() % 1001; //fgoal for target
    int goalY = rand() % 1001; //goal for target
    

    /***********************VARS TO CHANGE FOR TESTS*******************************************/
    int numSensors = 100;  //NUMBER OF SENSOR NODES 100-250
    double distanceToSend = 100.0; // DISTANCE THE MESSAGE WILL TRAVEL IN METERS
    int timeInterval = 5; // T seconds WILL BE CHANGED 
    GatewayNode *newGateway = new GatewayNode(gX,gY,numSensors,true);
    TargetNode *newTarget = new TargetNode(tX,tY);
    //create the setup of the simulation



    Simulation mySimulation(numSensors, newGateway, distanceToSend, timeInterval, mySensors,newTarget);
    
    //this timer is to switch the sensors modes to on and off
    bool timerFlag = true; 
    bool swap = true; //all sensors start at true 
    thread moveThread; //create a thread to move target node 
    thread eSensing; //thread to run the eSensing function
    thread transmitThread; //create a thread for packet exhange

    thread timer([&]() {
        while (timerFlag) {
            if(swap == true){
                this_thread::sleep_for(chrono::seconds(2));  // nothing for 2 seconds
                mySimulation.switchBools(numSensors);
            }
            this_thread::sleep_for(chrono::seconds(timeInterval));
            mySimulation.switchBools(numSensors);
            swap = false;
        }
    });

    //drop nodes into place
    if(mySimulation.addNodes(mySensors, numSensors) == true){
        cout << "Simulation begin: "<< startTime  << endl;
        cout << endl;
        mySimulation.dumpSim();
    }
    else{
        cout << "failure" << endl;
    }

    cout << endl;
    moveThread = thread([&]() {
        cout << "Target Node starts at: ( " << tX << "," << tY <<  " )" << endl;
        cout << "Target Node now moving..." << endl;
        cout << endl;
        while (timerFlag) {
            //call the senseTarget function
            newTarget->moveToGoal(goalX, goalY);

            //sleep for a short duration to control the loop
            //Speed of Target
            this_thread::sleep_for(chrono::milliseconds(200));
        }
    });


    //once placed on grid sensors should start waiting sensing moving Target
    eSensing = thread([&]() {
        cout << "Sensing thread started..." << endl;
        cout << endl;
        while (timerFlag) {
            //call the senseTarget function
            mySimulation.senseTarget();
            //sleep for a short duration to control the loop
            this_thread::sleep_for(chrono::milliseconds(500));
        }
    });

    //to see the location of the Gateway node
    cout << "Gateway is at (" << gX << "," << gY << ")"<< endl;
    cout << endl;
    //mySimulation.dumpSim();

    cout << "All Connections:" << endl;
    cout << "Node paths deemed INVALID are considered irrelevant" << endl;
    cout << endl;
    for(int i = 0; i < numSensors; i++){
        double dist = mySimulation.shortestDist(mySensors.at(i),0);
        if((dist < 1000.0) && (dist != -1)){
            //if the distance is greater than the grid size
            //or shortest dist returns -1 that means it didnt reach the gateway node
            mySimulation.canTransmit(mySensors.at(i)); //this node can transmit
            mySensors.at(i).setDistance(dist);
            cout << i << " path is " << dist << " meters" << endl;
            cout << endl;
        }else{
            mySensors.at(i).setDistance(-1); //if path invalid set distance to -1
            cout << i << " path is INVALID " << endl;
            cout << endl;
        }

    }

    //now that path is found start transmitting data

    transmitThread = thread([&]() {
        cout << "Now transmitting..." << endl;
        cout << "Only nodes in best paths (m_canTransmit set to true) may transmit" << endl;
        cout << endl;
        int i = 0;
        while (timerFlag) {
            //go through and continuously transmit data to all nodes that can transmit (m_canTransmit == true)
            if(mySensors.at(i).getCanTransmit() == true){
                //call the e_Transmit & e_Recieve fxns to depleat energies until 1 of the nodes die 
                int dead = mySimulation.packetExchange(mySensors.at(i)); 
                //if packetExhange returns -1 packet is living 
                //if node dies then packetExchange returns that node ID
                if(dead >= 0){
                    time_t currentTime = time(0);
                    string endTime = ctime(&currentTime);
                    cout << "Node " << dead << " has died: " << endTime << endl;
                    cout << "Simulation ending..." << endl;
                    newTarget->endTarget();
                    timerFlag = false; //flag to stop the timer
                } 
            }
            if(i+1 == numSensors){
                //if get to end of the list reset i
                i = 0;
            }else{

                i++;
            }        
            //sleep for a short duration to control the loop
            this_thread::sleep_for(chrono::milliseconds(50));
        }
    });
    //lastly stop the timer
    timer.join();//wait for the timer thread to finish
    cout << "timer off" << endl;
    eSensing.join(); //end sensing
    cout << "sensing off" << endl;
    transmitThread.join();
    cout << "transmission off" << endl;
    moveThread.join();
    cout << "moving off" << endl;

    //deallocate memory
    delete newGateway;
    delete newTarget;
    cout << "Simulation has ended" << endl;
    return 0;
}
