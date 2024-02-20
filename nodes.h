#ifndef NODES_H
#define NODES_H
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <thread>
#include <chrono>


using namespace std;
//list of consts here
const int MAX_AREA = 1000; //max size of board is 1000x1000
const int PACKET_SIZE = 10000; //10 Kbit = 10000 bits
const int S_DATA_SIZE = 1000; //1Kbit = 1000 bits
const int INITIAL_ENERGY = 1; //initial energy is 1 joule
const double MAX_SENSOR_DISTANCE = 140.0; //sensor range is 140
const double A_NANO = 25 * 0.00000001;


//Use Dijkstra's algorithm to calculate least cost (minimum energy over head) path for each sensor
//sensing range is 140m so anything over that cannot be reached
//
//classes 
class GatewayNode;
class SensorNode;
class Simulation;

class GatewayNode{

    public:
    friend class SensorNode;
    friend class Simulation;
    GatewayNode(); //constructor
    GatewayNode(int x, int y,int sensors,bool gate){
        x_coordinate = x; //hold gateway x coordinate
        y_coordinate = y; //hold gaeway y coordinate
        m_numSensors = sensors; //number of sesnors
        m_isGateway = gate;
    }

    private:
    int x_coordinate;
    int y_coordinate; 
    int m_numSensors; //number of living nodes
    bool m_isGateway;

};

class SensorNode{
    public:
    friend class GatewayNode;
    friend class TargetNode;
    friend class Simulation;

    //default constructor
    SensorNode(){
        //if default constructor used
        //node will be deemed inactive 

        x_coordinate = 0; //x coor of node
        y_coordinate = 0; //y coor of node
        m_nodeID = 0; //id of node
        //m_targetID = 0; // id of the Gateway node
       //m_Tsec = 0; // t
        m_isActive = false; //inactive default nodes
        m_Sensing = false;
        m_canTransmit = false;
        m_next = nullptr; //null for now
        m_energy = 0;
        m_distance = 0.0;
        
    }

    //constructor
    SensorNode(int x, int y, int id
    , int gate,bool active,int energy){
        x_coordinate = x; //x coor of node
        y_coordinate = y; //y coor of node
        m_nodeID = id; //id of node
        m_Sensing = false;
        m_canTransmit = false;
        m_isActive = active;
        m_next = nullptr; //null for now
        m_energy = energy;
        m_distance = 0.0;
    }
    double e_Sensing(double energy){
        //calculates energy to senese
        //25 x 10^-8 joules conversion
        //as x q bits
        energy = energy - (A_NANO * PACKET_SIZE);
        return energy;
        
    }
    double e_Transmission(double energy,int distance){
        //calculates energy to Transmit data
        //at + a2d2 * b bits
        energy = energy - ((A_NANO + 50 * 0.000000000001 * (distance * distance)) * PACKET_SIZE);
        return energy;
    }
    double e_Recieve(double energy){
        energy = energy - (A_NANO * PACKET_SIZE);
        return energy;
    }
    //check if sensor active
    bool checkActive(){
        //returns the current state of sesnor
        return m_isActive;
    } 
    bool toggleActive(){
        if((m_isActive == true) && (m_Sensing == false)){
            //if sensor is active && not sensing then put to sleep mode
            m_isActive = false;
        }
        if(m_isActive == false){
            m_isActive = true;
        }
        return m_isActive;
    }
    bool toggleCanTransmit(){
        //node can transmit
        m_canTransmit = true;
        return m_canTransmit;
    }
    void setDistance(double distance){
        m_distance = distance;
    }
    bool getCanTransmit(){
        return m_canTransmit;
    }

    private:
    int x_coordinate;
    int y_coordinate;
    double m_energy; //energy of sensor
    int m_nodeID; //id for each node
    bool m_Sensing;
    bool m_isActive; //active/sleep
    bool m_canTransmit;
    double m_distance;
    SensorNode *m_next; //next node

};


class TargetNode{
    public:
    friend class SensorNode;
    friend class Simulation;

    TargetNode(int x, int y){
        x_coordinate = x;
        y_coordinate = y;
        m_isAlive = true;
    }

    bool moveToGoal(int goal_x, int goal_y) {
        //line equation: y = mx + b
        double slope = static_cast<double>(goal_y - y_coordinate) / (goal_x - x_coordinate);

        //call fxn to update the x and y coords to move towards goal
        bool stop = false;
        while ((stop == false) && (m_isAlive == true)) {
            // Move towards the goal
            moveTowardsGoal(slope, goal_x);

            //testing print the current position of gate
            //cout << "Target position: (" << x_coordinate << ", " << y_coordinate << ")" << endl;

            //constant speed 4-6 seconds (SUBJECT TO CHANGE)
            this_thread::sleep_for(chrono::seconds(4));
            if(m_isAlive == false){
                stop = true;
            }
        }
        return stop;
    }
    //keep the gate moving constanly
    void moveTowardsGoal(double slope, int goal_x) {
        //if x is not goal coordinate keep updating
        if (goal_x > x_coordinate) {
            ++x_coordinate;
            y_coordinate = static_cast<int>(slope * x_coordinate);
        } else if (goal_x < x_coordinate) {
            --x_coordinate;
            y_coordinate = static_cast<int>(slope * x_coordinate);
        }
    }
    void endTarget(){
        cout << "endTarget called" << endl;
        m_isAlive = false;
    }


    private:
    int x_coordinate;
    int y_coordinate;
    bool m_isAlive;
};

class Simulation{
    public:  
    // Constructor
    Simulation(int num, GatewayNode* gate, double distance, int T, vector<SensorNode>& mySensors,TargetNode *target)
        : m_distToSend(distance), m_Tsec(T), m_gateway(gate), m_numSensors(num), grid_size(MAX_AREA), sensors(mySensors),m_target(target) {}

    //used to add nodes in bulk during simulation
    bool addNodes(vector<SensorNode> &sensors,int num){
    bool success = false;
    srand(static_cast<unsigned>(time(nullptr))); //rand num gen
    int x = 0;
    int y = 0;
    for(int i = 0 ; i < num ;i++){
        bool flag = true; //assume coord exists
        SensorNode newNode;
        x = rand() % MAX_AREA;
        y = rand() % MAX_AREA; 
        while(flag == true){
            if(checkCoord(sensors,x,y) == false){
                newNode.x_coordinate = x;
                newNode.y_coordinate = y;
                flag = false;
            }
            else{
                //generate new coords if already exist
                 x = rand() % MAX_AREA; 
                 y = rand() % MAX_AREA;
            }
        }
        newNode.m_energy = INITIAL_ENERGY; //intial energy is 1 joule
        newNode.m_isActive = true; //activate node
        newNode.m_nodeID = i;
        newNode.m_next = nullptr;
        sensors.push_back(newNode);
    }
    if(int(sensors.size()) == num){
        success = true;
    }
        return success;
    }
    void switchBools(int numSensors){
        for(int i = 0; i < numSensors; i++){
            sensors.at(i).toggleActive();
        }
    }

    bool checkCoord(vector<SensorNode> &sensors,int x ,int y){
        for (const SensorNode& sensor : sensors) {
        if (sensor.x_coordinate == x && sensor.y_coordinate == y) {
            return true; // Coordinates already exist
        }
    }
        return false; // Coordinates not found
    }

    double calcDistance(int x1,int x2,int y1, int y2){
        return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
    }

    bool checkNearby(SensorNode &source, SensorNode &destination){
        bool isConnected = false;
        double dist = calcDistance(source.x_coordinate, destination.x_coordinate,source.y_coordinate,destination.y_coordinate);
        //make sure the distance is closer to gateway before connecting
        if(dist >= MAX_SENSOR_DISTANCE){
            //calculate distance between node 1 and 2 and ensure before connecting that it is closer to the gateway
            double gate = calcDistance(source.x_coordinate, m_gateway->x_coordinate,source.y_coordinate,m_gateway->y_coordinate);
            double gate2 = calcDistance(destination.x_coordinate, m_gateway->x_coordinate,destination.y_coordinate,m_gateway->y_coordinate);

            if(gate > gate2){
                isConnected = true;
            }
        }
        return isConnected;
    }
    double shortestDist(SensorNode &node1,double dist) {
        
        vector<SensorNode> path; //will collect nodes within range of node
        int j = 0;

        for (unsigned int i = 0; i < sensors.size(); ++i) {
            //will go thru and collect nodes within rnage of node1
            if (node1.m_nodeID != sensors.at(i).m_nodeID) {
                if (checkNearby(node1, sensors.at(i))) {
                    path.push_back(sensors.at(i));
                    j++;
                }
            }
        }
        //now it will find the best node to connect with as it has shortest distance
        double tmp = 1000;
        SensorNode best;
        for (unsigned int k = 0; k < path.size(); k++) {
            double cmp = calcDistance(node1.x_coordinate, path.at(k).x_coordinate, node1.y_coordinate, path.at(k).y_coordinate);
            if (tmp >= cmp) {
                best = path.at(k); //best sensor node to connect to
                tmp = cmp;
            }
        }
        //cout << "tmp =" << tmp;
        if(path.empty()){
            //if not in range of any sensorNodes it could be near gateway which is seperate
            double gate = calcDistance(node1.x_coordinate, m_gateway->x_coordinate,node1.y_coordinate,m_gateway->y_coordinate);

            if(gate <= MAX_SENSOR_DISTANCE){
                //if in range connect
                dist += gate;
                cout << node1.m_nodeID << " -> " << "GATEWAY" << endl;
                
                return dist;
            }
            else{
                //else no connections DISCARD
                cout << node1.m_nodeID << "-1 meters" << endl;
                return -1; 
                
            }
            
        }
        else{
            dist = tmp; //makes best distance dist var 
            cout << node1.m_nodeID << " -> ";
            dist += shortestDist(best,dist); //recurse and see if other can connect
        }
        return dist;  
    }
    void dumpConnections(SensorNode &node1){
        for(unsigned int i = 0; i < sensors.size();i++){
            if(node1.m_nodeID != sensors.at(i).m_nodeID){
                if(checkNearby(node1,sensors.at(i)) == true){
                    cout << "Nodes: " << node1.m_nodeID << " & " << sensors.at(i).m_nodeID << " can connect." << endl;
                }
                else{
                    i++;
                }
            }
            else{
                i++;
            }
        }
        double gate = calcDistance(node1.x_coordinate, m_gateway->x_coordinate,node1.y_coordinate,m_gateway->y_coordinate);

            if(gate <= MAX_SENSOR_DISTANCE){
                //if in range connect
                cout << node1.m_nodeID << " & GATEWAY can connect" << endl;
            }
        cout << endl;
    }
    void dumpSim(){
        //used to see the nodes and locations
        for (const SensorNode& sensor : sensors) {
            cout << "[" << sensor.m_nodeID << "] located: " 
            << "(" <<  sensor.x_coordinate << "," << sensor.y_coordinate << ")" << endl;
        }
    }

    void senseTarget(){
        
        //go thru the sensors to see if the target node is currently near the sensor
        for(int i = 0;i < m_numSensors;i++){
           double dist = calcDistance(sensors.at(i).x_coordinate,m_target->x_coordinate,sensors.at(i).y_coordinate,m_target->y_coordinate);
           //
           if(dist <= MAX_SENSOR_DISTANCE && (sensors.at(i).m_isActive == true)){
            sensors.at(i).m_Sensing = true; //change to note sensor is doing something so don't sleep
            sensors.at(i).m_energy = sensors.at(i).e_Sensing(sensors.at(i).m_energy);
            sensors.at(i).m_Sensing = false; //once done sensing turn off sensing bool
           }
        }
    }
    void canTransmit(SensorNode &node1){
        //allow  node to transmit data
        node1.toggleCanTransmit();
    }

    int packetExchange(SensorNode &node1){
        //go thru sensors
        for(int i = 0;i < m_numSensors;i++){
            if(i == node1.m_nodeID){
                //transmit and recieve data
                node1.m_energy = node1.e_Transmission(node1.m_energy,node1.m_distance);
                cout << node1.m_nodeID << " energy = " << node1.m_energy << endl;
                node1.m_energy = node1.e_Recieve(node1.m_energy);
                cout << node1.m_nodeID << " energy = " << node1.m_energy << endl;
                if(node1.m_energy <= 0){
                    //return the node that died first
                    return node1.m_nodeID;
                }
            }

        }
        return -1; //if no nodes are "dead the return -1"
    }

    private:
    double m_distToSend;
    int m_Tsec; //var val to use during tests , Tsec is how long before node turns back onconst
    GatewayNode *m_gateway; //gateway node
    int m_numSensors;
    int grid_size;
    vector<SensorNode> &sensors;
    TargetNode *m_target; //gate node to update
};

#endif