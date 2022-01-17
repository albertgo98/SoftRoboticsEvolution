//
//  main.cpp
//  EA_Robot_Controller
//
//  Algorithm that Coevolves Robot Shape and a Controller
//
//  Created by Albert Go on 11/30/21.
//

#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>
#include <list>
#include <algorithm>

using namespace std;

struct PointMass{
    double mass;
    vector<float> position; // {x, y, z}
    vector<float> velocity; // {v_x, v_y, v_z}
    vector<float> acceleration; // {a_x, a_y, a_z}
    vector<float> forces; // {f_x, f_y, f_z}
    int ID; //index of where a particular mass lies in the robot.masses vector https://forms.gle/bKtGGQKmbtsS6kSV7
    
};

struct Spring{
    float L0; // resting length
    float L; // current length
    float k; // spring constant
    int m0; // connected to which PointMass
    int m1; // connected to which PointMass
    float original_L0;
    int ID; //the index of where a particular string lies in the robot.springs vector
};

struct Cube{
    vector<PointMass> masses;
    vector<Spring> springs;
    vector<int> joinedCubes;
    vector<int> otherFaces; //faces of other cubes that are joined to it
    vector<int> joinedFaces; //faces of the cube that are joined to other cubes
    vector<int> massIDs; //where the verteces of the cube correspond to the Robot.masses vector
    vector<int> springIDs; //where the springs of the cube correspond to the Robot.springs vector
    vector<int> free_faces;
    vector<float> center;
};

struct Equation{
    float k;
    float a;
    float w;
    float c;
};

struct Controller{
    vector<Equation> motor;
    vector<float> start;
    vector<float> end;
    float fitness = 0;
};

struct Robot{
    vector<PointMass> masses; //vector of masses that make up the robot
    vector<Spring> springs; //vector of springs that make up the robot
    vector<int> cubes;
    vector<Cube> all_cubes;
    vector<int> available_cubes;
    float fitness = 0;
    Controller best_controller;
    vector<float> center;
};

const double g = -9.81; //acceleration due to gravity
const double b = 1; //damping (optional) Note: no damping means your cube will bounce forever
const float spring_constant = 5000.0f; //this worked best for me given my dt and mass of each PointMass
const float mu_s = 0.74; //coefficient of static friction
const float mu_k = 0.57; //coefficient of kinetic friction
float T = 0.0;
float dt = 0.0001;
bool breathing = true;

const int cut_point1 = 5;
const int cut_point2 = 10;

vector<int> face0 = {0, 1, 2, 3}; //face 0 (bottom face) corresponds with these cube vertices; only connects with face 5
vector<int> face1 = {0, 3, 4, 7}; //face 1(front face) corresponds with these cube vertices; only connects with face 3
vector<int> face2 = {0, 1, 4, 5}; //face 2 (left face) corresponds with these cube vertices; only connects with face 4
vector<int> face3 = {1, 2, 5, 6}; //face 3 (back face) corrresponds with these cube vertices; only connects with face 1
vector<int> face4 = {3, 2, 7, 6}; //face 4 (right face) corresponds with these cube vertices; only conncects with face 2
vector<int> face5 = {4, 5, 6, 7}; //face 5 (top face) corresponds with these cube vertices; only connects with face 0

vector<int> face0_springs = {0, 1, 2, 3, 4, 5}; //face 0 (bottom face) corresponds with these cube springs; only connects with face 5
vector<int> face1_springs = {3, 6, 9, 10, 11, 21}; //face 1 (front face) corresponds with these cube springs; only connects with face 3
vector<int> face2_springs = {0, 6, 7, 12, 13, 18}; //face 2 (left face) corresponds with these cube springs; only connects with face 4
vector<int> face3_springs = {1, 7, 8, 14, 15, 19}; //face 3 (back face) corresponds with these cube springs; only connects with face 1
vector<int> face4_springs = {2, 9, 8, 17, 16, 20}; //face 4 (right face) corresponds with these cube springs; only connects with face 2
vector<int> face5_springs = {18, 19, 20, 21, 22, 23}; // face 5 (top face) corresponds with these cube springs; only connects with face 0

vector<float> const_k = {1000, 5000, 5000, 10000};
vector<float> const_a = {0.1, 0.12, 0.15};
vector<float> const_w = {M_PI, 2*M_PI};
vector<float> const_c = {0, M_PI};

void initialize_masses(vector<PointMass> &masses);
void initialize_springs(vector<Spring> &springs);
void apply_force(vector<PointMass> &masses);
void update_pos_vel_acc(Robot &robot);
void update_forces(Robot &robot);
void reset_forces(Robot &robot);
void update_breathing(Robot &robot, Controller &control);
void initialize_robot(Robot &robot);
void initialize_cube(Cube &cube);
void fuse_faces(Cube &cube1, Cube &cube2, int cube1_index, int cube2_index, vector<PointMass> &masses, vector<Spring> &springs, int combine1, int combine2, vector<int> &masses_left, vector<int> &springs_left);
void breed_robots(vector<Robot> &new_robot_population, Robot &robot1, Robot &robot2, vector<Controller> &population, vector<Controller> &major_league);
void get_population(vector<Controller> &population, vector<Robot> &robot_population);
void replenish_population(vector<Controller> &new_set, vector<Robot> &robot_population);
void create_equation(Controller &control);
void mutate(Controller &offspring);
bool compareByFitness(const Controller &control1, const Controller &control2);
float determine_fitness(Controller &control, Robot robot);
void breed(vector<Controller> &new_population, Controller control1, Controller control2, vector<Robot> &robot_population);
void get_robot_population(vector<Robot> &robot_population);
bool compareByFitnessR(const Robot &robot1, const Robot &robot2);
void replenish_robot_population(vector<Robot> &new_robot_set, vector<Controller> &population, vector<Controller> &major_league);


int main(int argc, const char * argv[]) {
    // insert code here...
    srand( static_cast<unsigned int>(time(0)));
    std::cout << "Hello, World!\n";
    
    vector<Robot> robot_population;
    
    get_robot_population(robot_population);
    
    for (int q=0; q< robot_population.size(); q++){
        float x_center = 0;
        float y_center = 0;
        float z_center = 0;
        for (int m=0; m<robot_population[q].masses.size(); m++){
            x_center += robot_population[q].masses[m].position[0];
            y_center += robot_population[q].masses[m].position[1];
            z_center += robot_population[q].masses[m].position[2];
        }
        
        x_center = x_center/robot_population[q].masses.size();
        y_center = y_center/robot_population[q].masses.size();
        z_center = z_center/robot_population[q].masses.size();
        
        robot_population[q].center = {x_center, y_center, z_center};
    }
    
    cout<< "Initialized Robot Population" << endl;
    
    vector<Controller> population;
    vector<Controller> major_league;
    vector<Controller> update_major;
    
    get_population(population, robot_population);
    sort(population.begin(), population.end(), compareByFitness);
    sort(robot_population.begin(), robot_population.end(), compareByFitnessR);
    cout<< "Initialized Controller Population" << endl;
    
    int evaluations = 0;
    
    // Evolution loop
    while(evaluations < 1000)
    {
        
        vector<Robot> new_robot_population;
        vector<Controller> new_population;
        vector<Controller> new_major;
        cout << population.size();
        cout << ", ";
        cout << major_league.size() << endl;
        
        if (evaluations % 2 == 0){
            cout << "Evolving Robots Now" << endl;
            for (int r=0; r<robot_population.size(); r++){
                int parent2 = rand() % 10;
                if(parent2 == r){
                    bool same = true;
                    while(same){
                        parent2 = rand() % 10;
                        if(parent2 != r){
                            same = false;
                        }
                    }
                }
                breed_robots(new_robot_population, robot_population[r], robot_population[parent2], population, major_league);
            }
            robot_population = new_robot_population;
            sort(robot_population.begin(), robot_population.end(), compareByFitnessR);
        }
        else{
            cout << "Evolving Controller Now" << endl;
            for (int i=0; i<population.size(); i++){
                int parent2 = rand() % 50;
                if(parent2 == i){
                    bool same = true;
                    while(same){
                        parent2 = rand() % 50;
                        if(parent2 != i){
                            same = false;
                        }
                    }
                }
                breed(new_population, population[i], population[parent2], robot_population);
                if (i < major_league.size() && major_league.size() > 0){
                    int parent2_m2 = rand() % 25;
                    if(parent2_m2 == i){
                        bool same = true;
                        while(same){
                            parent2_m2 = rand() % 25;
                            if(parent2_m2 != i){
                                same = false;
                            }
                        }
                    }
                    breed(new_major, major_league[i], major_league[parent2_m2], robot_population);
                }
            }
            population = new_population;
            major_league = new_major;
        }
        
        sort(population.begin(), population.end(), compareByFitness);
        sort(major_league.begin(), major_league.end(), compareByFitness);
        
        sort(robot_population.begin(), robot_population.end(), compareByFitnessR);
    
        evaluations += 1;
        
        if (evaluations % 10 == 0){
            //UPDATING CONTROLLER LITTLE LEAGUE AND MAJOR LEAGUE AND REPLENISHING CONTROLLER POPULATION
            //-----------------------------------------------------------------------------------------
            if (major_league.size() > 0){
                major_league.erase(major_league.begin()+12, major_league.end());
            }
            
            update_major = {population.begin(), population.begin()+25};
            major_league.insert(major_league.end(), update_major.begin(), update_major.end());
            
            population.erase(population.begin(), population.begin()+25);
            
            vector<Controller> new_set;
            replenish_population(new_set, robot_population);
            
            population.insert(population.end(), new_set.begin(), new_set.end());
            //-----------------------------------------------------------------------------------------
            
            //UPDATING ROBOT POPULATION; TAKING OUT THE LEAST FIT AND REPLACING THEM RANDOMLY
            //-----------------------------------------------------------------------------------------
            
            robot_population.erase(robot_population.begin()+5, robot_population.end());
            
            vector<Robot> new_robot_set;
            replenish_robot_population(new_robot_set, population, major_league);
            
            robot_population.insert(robot_population.end(), new_robot_set.begin(), new_robot_set.end());
            //-----------------------------------------------------------------------------------------
            
            sort(population.begin(), population.end(), compareByFitness);
            sort(major_league.begin(), major_league.end(), compareByFitness);
            
            sort(robot_population.begin(), robot_population.end(), compareByFitnessR);
        }
        
        for (int s=0; s < robot_population.size(); s++){
            cout << "ROBOT NUMBER = ";
            cout << s << endl;
            
            cout << "ROBOT FITNESS = ";
            cout << robot_population[s].fitness << endl;
            
            cout << "CONTROLLER = ";
            cout << "< ";
            for (int j=0; j<robot_population[s].best_controller.motor.size(); j++){
                cout << "[";
                cout << robot_population[s].best_controller.motor[j].k;
                cout << ", ";
                cout << robot_population[s].best_controller.motor[j].a;
                cout << ", ";
                cout << robot_population[s].best_controller.motor[j].w;
                cout << ", ";
                cout << robot_population[s].best_controller.motor[j].c;
                cout << "]";
                cout << ", ";
            }
            cout << "> " << endl;
            
            cout << "ROBOT" << endl;
            cout << "-------------" << endl;
            for (int l=0; l<robot_population[0].all_cubes.size(); l++){
                cout << "Cube Number = ";
                cout << l << endl;
                cout << "Fused to Cube = ";
                for (int n=0; n<robot_population[0].all_cubes[l].joinedCubes.size(); n++){
                    cout << robot_population[0].all_cubes[l].joinedCubes[n];
                    if (n == robot_population[0].all_cubes[l].joinedCubes.size()-1){
                        cout << "; " << endl;
                    }
                    else{
                        cout << ", ";
                    }
                }
                cout << "Its faces fused = ";
                for (int n=0; n<robot_population[0].all_cubes[l].joinedFaces.size(); n++){
                    cout << robot_population[0].all_cubes[l].joinedFaces[n];
                    if (n == robot_population[0].all_cubes[l].joinedFaces.size()-1){
                        cout << "; " << endl;
                    }
                    else{
                        cout << ", ";
                    }
                }
                cout << "************" << endl;
            }
            cout << "-------------" << endl;
        }
        cout << "FINISHED PRINTING OUT ROBOTS" << endl;
        
        cout << "EVALUATIONS = ";
        cout << evaluations << endl;
    }

    
    return 0;
}

//EVOLVING CONTROLLER HERE
//-----------------------------------------------------------------------
void get_population(vector<Controller> &population, vector<Robot> &robot_population){
    int individuals = 0;
    
    while (individuals < 50) {
        cout << "New Controller" << endl;
        Controller control;
        create_equation(control);
        
        for (int r=0; r<robot_population.size(); r++){
            cout << "Testing Robot = ";
            cout << r << endl;
            float x_center = 0;
            float y_center = 0;
            float z_center = 0;
            for (int m=0; m<robot_population[r].masses.size(); m++){
                x_center += robot_population[r].masses[m].position[0];
                y_center += robot_population[r].masses[m].position[1];
                z_center += robot_population[r].masses[m].position[2];
            }
            
            x_center = x_center/robot_population[r].masses.size();
            y_center = y_center/robot_population[r].masses.size();
            z_center = z_center/robot_population[r].masses.size();
            
            control.start = {x_center, y_center, z_center};
            
            float f = determine_fitness(control, robot_population[r]);
            
            if (f > control.fitness){
                control.fitness = f;
            }
            if (f > robot_population[r].fitness){
                robot_population[r].fitness = f;
                robot_population[r].best_controller = control;
            }
        }
        
        cout << "Fitness = ";
        cout << control.fitness << endl;
        
        population.push_back(control);
        individuals += 1;
    }
}

void replenish_population(vector<Controller> &new_set, vector<Robot> &robot_population){
    int individuals = 0;
    
    while (individuals < 25) {
        cout << "Replenishing Controller Population..." << endl;
        Controller control;
        create_equation(control);
        
        for (int r=0; r<robot_population.size(); r++){
            float x_center = 0;
            float y_center = 0;
            float z_center = 0;
            for (int m=0; m<robot_population[r].masses.size(); m++){
                x_center += robot_population[r].masses[m].position[0];
                y_center += robot_population[r].masses[m].position[1];
                z_center += robot_population[r].masses[m].position[2];
            }
            
            x_center = x_center/robot_population[r].masses.size();
            y_center = y_center/robot_population[r].masses.size();
            z_center = z_center/robot_population[r].masses.size();
            
            cout << "Center = ";
            cout << x_center;
            cout << ", ";
            cout << y_center;
            cout << ", ";
            cout << z_center << endl;
            
            control.start = {x_center, y_center, z_center};
            
            float f = determine_fitness(control, robot_population[r]);
            
            if (f > control.fitness){
                control.fitness = f;
            }
            if (f > robot_population[r].fitness){
                robot_population[r].fitness = f;
                robot_population[r].best_controller = control;
            }
        }
        cout << "Fitness = ";
        cout << control.fitness << endl;
        
        new_set.push_back(control);
        individuals += 1;
    }
}

void create_equation(Controller &control){
    for (int i=0; i<14; i++){
        Equation eqn;
        int rand1 = rand() % 4;
        int rand2 = rand() % 3;
        int rand3 = rand() % 2;
        int rand4 = rand() % 2;
        
        eqn.k = const_k[rand1];
        if (rand1 == 0){
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (rand1 == 3){
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else{
            eqn.a = const_a[rand2];
            eqn.w = const_w[rand3];
            eqn.c = const_c[rand4];
        }
        
        control.motor.push_back(eqn);
    }
}

float determine_fitness(Controller &control, Robot robot){
    float displacement = 0;
    int runs = 0;
    T = 0;
    
    while (runs < 300){
        
        //Let's test the controller
        //-------------------------------------
        for (int k=0; k<50; k++){
            T = T + dt; //update time that has passed
            if (breathing) {
                update_breathing(robot, control);
            }

            update_forces(robot);
            update_pos_vel_acc(robot);
            
            reset_forces(robot);
            
        }
        //-------------------------------------
        
        runs += 1;
    }
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    for (int m=0; m<robot.masses.size(); m++){
        x_center += robot.masses[m].position[0];
        y_center += robot.masses[m].position[1];
        z_center += robot.masses[m].position[2];
    }
    
    x_center = x_center/robot.masses.size();
    y_center = y_center/robot.masses.size();
    z_center = z_center/robot.masses.size();
    
    control.end = {x_center, y_center, z_center};
    
    displacement = sqrt(pow(control.end[0]-control.start[0], 2) + pow(control.end[1]-control.start[1], 2));
    
    return displacement;
}

void breed(vector<Controller> &new_population, Controller control1, Controller control2, vector<Robot> &robot_population){
    Controller offspring;
    
    bool recomb = false;
    for (int i=0; i<14; i++){
        if (i==cut_point1){
            recomb = true;
        }
        else if (i==cut_point2){
            recomb = false;
        }
        if (recomb){
            offspring.motor.push_back(control2.motor[i]);
        }
        else{
            offspring.motor.push_back(control1.motor[i]);
        }
    }
    
    
    int rand1 = rand() % 100;
    
    if (rand1 < 50){
        mutate(offspring);
    }
    
    for (int r=0; r<robot_population.size(); r++) {
        float x_center = 0;
        float y_center = 0;
        float z_center = 0;
        for (int m=0; m<robot_population[r].masses.size(); m++){
            x_center += robot_population[r].masses[m].position[0];
            y_center += robot_population[r].masses[m].position[1];
            z_center += robot_population[r].masses[m].position[2];
        }
        
        x_center = x_center/robot_population[r].masses.size();
        y_center = y_center/robot_population[r].masses.size();
        z_center = z_center/robot_population[r].masses.size();
        
        offspring.start = {x_center, y_center, z_center};
        
        float f = determine_fitness(offspring, robot_population[r]);
        
        if (f > offspring.fitness){
            offspring.fitness = f;
        }
        if (f > robot_population[r].fitness){
            robot_population[r].fitness = f;
            robot_population[r].best_controller = offspring;
        }
    }
    
    if (offspring.fitness > control1.fitness){
        new_population.push_back(offspring);
    }
    else{
        new_population.push_back(control1);
    }
    
}

void mutate(Controller &offspring){
    int rand_num = rand() % 14;
    int rand_num2 = rand() % 14;
    if(rand_num2 == rand_num){
        bool same = true;
        while(same){
            rand_num2 = rand() % 14;
            if(rand_num2 != rand_num){
                same = false;
            }
        }
    }
    
    iter_swap(offspring.motor.begin()+rand_num, offspring.motor.begin()+rand_num2);
}
//-----------------------------------------------------------------------

//POSITION, FORCE CALCULATIONS, AND CONTROLLER IMPLEMENTATION OCCUR HERE AND BELOW
//-----------------------------------------------------------------------
void update_breathing(Robot &robot, Controller &control){
    for (int i=0; i<robot.all_cubes.size(); i++){
        int ind0 = robot.all_cubes[i].springIDs[0];
        int ind1 = robot.all_cubes[i].springIDs[1];
        int ind2 = robot.all_cubes[i].springIDs[2];
        int ind3 = robot.all_cubes[i].springIDs[3];
        int ind4 = robot.all_cubes[i].springIDs[4];
        int ind5 = robot.all_cubes[i].springIDs[5];
        int ind6 = robot.all_cubes[i].springIDs[6];
        int ind7 = robot.all_cubes[i].springIDs[7];
        int ind8 = robot.all_cubes[i].springIDs[8];
        int ind9 = robot.all_cubes[i].springIDs[9];
        int ind10 = robot.all_cubes[i].springIDs[10];
        int ind11 = robot.all_cubes[i].springIDs[11];
        int ind12 = robot.all_cubes[i].springIDs[12];
        int ind13 = robot.all_cubes[i].springIDs[13];
        int ind14 = robot.all_cubes[i].springIDs[14];
        int ind15 = robot.all_cubes[i].springIDs[15];
        int ind16 = robot.all_cubes[i].springIDs[16];
        int ind17 = robot.all_cubes[i].springIDs[17];
        int ind18 = robot.all_cubes[i].springIDs[18];
        int ind19 = robot.all_cubes[i].springIDs[19];
        int ind20 = robot.all_cubes[i].springIDs[20];
        int ind21 = robot.all_cubes[i].springIDs[21];
        int ind22 = robot.all_cubes[i].springIDs[22];
        int ind23 = robot.all_cubes[i].springIDs[23];
        int ind24 = robot.all_cubes[i].springIDs[24];
        int ind25 = robot.all_cubes[i].springIDs[25];
        int ind26 = robot.all_cubes[i].springIDs[26];
        int ind27 = robot.all_cubes[i].springIDs[27];

        float k = control.motor[i].k;
        float a = control.motor[i].a;
        float w = control.motor[i].w;
        float c = control.motor[i].c;

        for (int k=0; k<28; k++){
            if (robot.all_cubes[i].springs[k].ID == ind0){
                robot.springs[ind0].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind1){
                robot.springs[ind1].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind2){
                robot.springs[ind2].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind3){
                robot.springs[ind3].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind4){
                robot.springs[ind4].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind5){
                robot.springs[ind5].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind6){
                robot.springs[ind6].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind7){
                robot.springs[ind7].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind8){
                robot.springs[ind8].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind9){
                robot.springs[ind9].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind10){
                robot.springs[ind10].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind11){
                robot.springs[ind11].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind12){
                robot.springs[ind12].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind13){
                robot.springs[ind13].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind14){
                robot.springs[ind14].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind15){
                robot.springs[ind15].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind16){
                robot.springs[ind16].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind17){
                robot.springs[ind17].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind18){
                robot.springs[ind18].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind19){
                robot.springs[ind19].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind20){
                robot.springs[ind20].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind21){
                robot.springs[ind21].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind22){
                robot.springs[ind22].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind23){
                robot.springs[ind23].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind24){
                robot.springs[ind24].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind25){
                robot.springs[ind25].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind26){
                robot.springs[ind26].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
            else if (robot.all_cubes[i].springs[k].ID == ind27){
                robot.springs[ind27].L0 = robot.all_cubes[i].springs[k].original_L0 + a*sin(w*T+c);;
            }
        }

        robot.springs[ind0].k = k;
        robot.springs[ind1].k = k;
        robot.springs[ind2].k = k;
        robot.springs[ind3].k = k;
        robot.springs[ind4].k = k;
        robot.springs[ind5].k = k;
        robot.springs[ind6].k = k;
        robot.springs[ind7].k = k;
        robot.springs[ind8].k = k;
        robot.springs[ind9].k = k;
        robot.springs[ind10].k = k;
        robot.springs[ind11].k = k;
        robot.springs[ind12].k = k;
        robot.springs[ind13].k = k;
        robot.springs[ind14].k = k;
        robot.springs[ind15].k = k;
        robot.springs[ind16].k = k;
        robot.springs[ind17].k = k;
        robot.springs[ind18].k = k;
        robot.springs[ind19].k = k;
        robot.springs[ind20].k = k;
        robot.springs[ind21].k = k;
        robot.springs[ind22].k = k;
        robot.springs[ind23].k = k;
        robot.springs[ind24].k = k;
        robot.springs[ind25].k = k;
        robot.springs[ind26].k = k;
        robot.springs[ind27].k = k;
    }
}

void update_pos_vel_acc(Robot &robot){
    
    for (int i=0; i<robot.masses.size(); i++){
        float acc_x = robot.masses[i].forces[0]/robot.masses[i].mass;
        float acc_y = robot.masses[i].forces[1]/robot.masses[i].mass;
        float acc_z = robot.masses[i].forces[2]/robot.masses[i].mass;

        robot.masses[i].acceleration[0] = acc_x;
        robot.masses[i].acceleration[1] = acc_y;
        robot.masses[i].acceleration[2] = acc_z;
        
        float vel_x = acc_x*dt + robot.masses[i].velocity[0];
        float vel_y = acc_y*dt + robot.masses[i].velocity[1];
        float vel_z = acc_z*dt + robot.masses[i].velocity[2];
        
        
        robot.masses[i].velocity[0] = vel_x*b;
        robot.masses[i].velocity[1] = vel_y*b;
        robot.masses[i].velocity[2] = vel_z*b;
        
        float pos_x = (vel_x*dt) + robot.masses[i].position[0];
        float pos_y = (vel_y*dt) + robot.masses[i].position[1];
        float pos_z = (vel_z*dt) + robot.masses[i].position[2];
        
        robot.masses[i].position[0] = pos_x;
        robot.masses[i].position[1] = pos_y;
        robot.masses[i].position[2] = pos_z;
    }
    
    for (int j=0; j<robot.all_cubes.size(); j++){
        int ind0 = robot.all_cubes[j].massIDs[0];
        int ind1 = robot.all_cubes[j].massIDs[1];
        int ind2 = robot.all_cubes[j].massIDs[2];
        int ind3 = robot.all_cubes[j].massIDs[3];
        int ind4 = robot.all_cubes[j].massIDs[4];
        int ind5 = robot.all_cubes[j].massIDs[5];
        int ind6 = robot.all_cubes[j].massIDs[6];
        int ind7 = robot.all_cubes[j].massIDs[7];
        
        for (int k=0; k<8; k++){
            if (robot.all_cubes[j].masses[k].ID == ind0){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind0].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind0].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind0].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind1){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind1].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind1].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind1].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind2){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind2].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind2].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind2].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind3){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind3].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind3].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind3].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind4){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind4].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind4].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind4].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind5){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind5].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind5].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind5].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind6){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind6].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind6].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind6].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind7){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind7].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind7].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind7].position[2];
            }
        }
    }
}

void reset_forces(Robot &robot){
    for(int i=0; i<robot.masses.size(); i++){
        robot.masses[i].forces = {0.0f, 0.0f, 0.0f};
    }
}

void update_forces(Robot &robot){
    
    for (int i=0; i<robot.springs.size(); i++){

        int p0 = robot.springs[i].m0;
        int p1 = robot.springs[i].m1;

        vector<float> pos0 = robot.masses[p0].position;
        vector<float> pos1 = robot.masses[p1].position;

        float spring_length = sqrt(pow(pos1[0]-pos0[0], 2) + pow(pos1[1]-pos0[1], 2) + pow(pos1[2]-pos0[2], 2));

        robot.springs[i].L = spring_length;
        float force = -robot.springs[i].k*(spring_length-robot.springs[i].L0);

        float x_univ = (pos0[0]-pos1[0])/spring_length;
        float y_univ = (pos0[1]-pos1[1])/spring_length;
        float z_univ = (pos0[2]-pos1[2])/spring_length;
        vector<float> force_unit_dir_2_1 = {x_univ,y_univ,z_univ};
        vector<float> force_unit_dir_1_2 = {-x_univ,-y_univ,-z_univ};

        for (int n = 0; n < 3; n++) {
            robot.masses[p0].forces[n] =  robot.masses[p0].forces[n] + force * force_unit_dir_2_1[n];
            robot.masses[p1].forces[n] =  robot.masses[p1].forces[n] + force * force_unit_dir_1_2[n];
        }
    }
    
    for (int j=0; j<robot.masses.size(); j++){
        robot.masses[j].forces[2] = robot.masses[j].forces[2] + robot.masses[j].mass*g;
        
        if (robot.masses[j].position[2] < 0){
            robot.masses[j].forces[2] = -robot.masses[j].position[2]*1000000.0f;
        }
        
        float F_n = robot.masses[j].mass*g;

        float F_h = sqrt(pow(robot.masses[j].forces[0], 2) + pow(robot.masses[j].forces[1], 2));


        if (F_n < 0){
            if (F_h < -F_n*mu_s){
                robot.masses[j].forces[0] = 0;
                robot.masses[j].forces[1] = 0;
            }
            if (F_h >= -F_n*mu_s){
                if (robot.masses[j].forces[0] > 0){
                    robot.masses[j].forces[0] = robot.masses[j].forces[0] + mu_k*F_n;
                }
                else{
                    robot.masses[j].forces[0] = robot.masses[j].forces[0] - mu_k*F_n;
                }
                if (robot.masses[j].forces[1] > 0){
                    robot.masses[j].forces[1] = robot.masses[j].forces[1] + mu_k*F_n;
                }
                else{
                    robot.masses[j].forces[1] = robot.masses[j].forces[1] - mu_k*F_n;
                }
            }
        }
    }
}
// ----------------------------------------------------------------------

//BREEDING ROBOTS OCCURS HERE!!
// ----------------------------------------------------------------------
void get_robot_population(vector<Robot> &robot_population){
    int individuals = 0;
    
    while (individuals < 10) {
        cout << "New Robot" << endl;
        Robot robot;
        initialize_robot(robot);
        
        robot_population.push_back(robot);
        individuals += 1;
    }
}

void replenish_robot_population(vector<Robot> &new_robot_set, vector<Controller> &population, vector<Controller> &major_league){
    int individuals = 0;
    
    while (individuals < 5) {
        cout << "New Robot" << endl;
        Robot robot;
        initialize_robot(robot);
        
        float x_center = 0;
        float y_center = 0;
        float z_center = 0;
        for (int m=0; m<robot.masses.size(); m++){
            x_center += robot.masses[m].position[0];
            y_center += robot.masses[m].position[1];
            z_center += robot.masses[m].position[2];
        }
        
        x_center = x_center/robot.masses.size();
        y_center = y_center/robot.masses.size();
        z_center = z_center/robot.masses.size();
        
        robot.center = {x_center, y_center, z_center};
        
        for (int c=0; c<population.size(); c++){
            population[c].start = robot.center;
            float f = determine_fitness(population[c], robot);
            if (f > robot.fitness){
                robot.fitness = f;
                robot.best_controller = population[c];
            }
            if (f > population[c].fitness){
                population[c].fitness = f;
            }
        }
        
        if (major_league.size() > 0){
            for (int m=0; m<major_league.size(); m++){
                major_league[m].start = robot.center;
                float f = determine_fitness(major_league[m], robot);
                if (f > robot.fitness){
                    robot.fitness = f;
                    robot.best_controller = major_league[m];
                }
                if (f > major_league[m].fitness){
                    major_league[m].fitness = f;
                }
            }
        }
        
        new_robot_set.push_back(robot);
        individuals += 1;
    }
}

void breed_robots(vector<Robot> &new_robot_population, Robot &robot1, Robot &robot2, vector<Controller> &population, vector<Controller> &major_league){
    Robot offspring;
    vector<PointMass> masses;
    vector<Spring> springs;
    vector<int> cubes;
    vector<Cube> all_cubes; //initializes all the cubes that will make up this robot
    vector<int> available_cubes;
    
    for (int i=0; i<14; i++){
        Cube cube;
        initialize_cube(cube); //initialize the cube
        if (i<5 || i>9){
            if (i==0){
                for (int j=0; j<28; j++){
                    cube.springs[j].ID = j;
                    cube.springIDs.push_back(j);
                    springs.push_back(cube.springs[j]);
                }
                for (int k=0; k<8; k++){
                    cube.masses[k].ID = k;
                    cube.massIDs.push_back(k);
                    masses.push_back(cube.masses[k]);
                }
                available_cubes.push_back(i);
            }
            else{
                int cube1 = robot1.all_cubes[i].joinedCubes[0];
                int cube2_face2 = robot1.all_cubes[i].joinedFaces[0];
                int cube1_face1;
                
                vector<int> map1;
                vector<int> map2;
                vector<int> masses_left;
                vector<int> springs_left;
                
                for (int s=0; s<8; s++){
                    masses_left.push_back(s);
                }
                
                for (int v=0; v<28; v++){
                    springs_left.push_back(v);
                }
                
                if (cube2_face2 == 0){
                    cube1_face1 = 5;
                    
                    map2 = face0;
                    map1 = face5;
                }
                else if (cube2_face2 == 5){
                    cube1_face1 = 0;
                    
                    map2 = face5;
                    map1 = face0;
                }
                else if (cube2_face2 == 1){
                    cube1_face1 = 3;
                    
                    map2 = face1;
                    map1 = face3;
                }
                else if (cube2_face2 == 3){
                    cube1_face1 = 1;
                    
                    map2 = face3;
                    map1 = face1;
                }
                else if (cube2_face2 == 2){
                    cube1_face1 = 4;
                    
                    map2 = face2;
                    map1 = face4;
                }
                else{
                    cube1_face1 = 2;
                    
                    map2 = face4;
                    map1 = face2;
                }
                
                if (find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1) == all_cubes[cube1].free_faces.end()){
                    bool clashing = true;
                    cout << "CLASHING" << endl;
                    while (clashing) {
                        int itr6 = find(all_cubes[cube1].joinedFaces.begin(), all_cubes[cube1].joinedFaces.end(), cube1_face1)-all_cubes[cube1].joinedFaces.begin();
                        cube1 = all_cubes[cube1].joinedCubes[itr6];
                        if (find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1) != all_cubes[cube1].free_faces.end()){
                            clashing = false;
                        }
                    }
                    cout << "RESOLVED" << endl;
                }
                
                int itr = find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1)-all_cubes[cube1].free_faces.begin();
                int itr2 = find(cube.free_faces.begin(), cube.free_faces.end(), cube2_face2)-cube.free_faces.begin();
                
                all_cubes[cube1].free_faces.erase(all_cubes[cube1].free_faces.begin()+itr);
                cube.free_faces.erase(cube.free_faces.begin()+itr2);
                
                float cube1_z0 = all_cubes[cube1].masses[0].position[2];
                
                if (cube1_face1 == 0 && cube1_z0 == 0){
                    float x_disp = all_cubes[cube1].masses[map1[0]].position[0]-cube.masses[map2[0]].position[0]; //x displacement
                    float y_disp = all_cubes[cube1].masses[map1[0]].position[1]-cube.masses[map2[0]].position[1]; //y displacement
                    float z_disp = all_cubes[cube1].masses[map1[0]].position[2]-cube.masses[map2[0]].position[2]; //z displacement
                    
                    for (int m=0; m<all_cubes.size(); m++){
                        for (int n=0; n<8; n++){
                            //shift cube 2 over
                            all_cubes[m].masses[n].position[0] -= x_disp;
                            all_cubes[m].masses[n].position[1] -= y_disp;
                            all_cubes[m].masses[n].position[2] -= z_disp;
                            
                            masses[all_cubes[m].masses[n].ID].position[0] = all_cubes[m].masses[n].position[0];
                            masses[all_cubes[m].masses[n].ID].position[1] = all_cubes[m].masses[n].position[1];
                            masses[all_cubes[m].masses[n].ID].position[2] = all_cubes[m].masses[n].position[2];
                        }
                        all_cubes[m].center[0] -= x_disp;
                        all_cubes[m].center[1] -= y_disp;
                        all_cubes[m].center[2] -= z_disp;
                    }
                }
                else{
                    //find where the second cube needs to join the first cube
                    float x_disp = cube.masses[map2[0]].position[0]-all_cubes[cube1].masses[map1[0]].position[0]; //x displacement
                    float y_disp = cube.masses[map2[0]].position[1]-all_cubes[cube1].masses[map1[0]].position[1]; //y displacement
                    float z_disp = cube.masses[map2[0]].position[2]-all_cubes[cube1].masses[map1[0]].position[2]; //z displacement
                    
                    for (int u=0; u<8; u++){
                        //shift cube 2 over
                        cube.masses[u].position[0] -= x_disp;
                        cube.masses[u].position[1] -= y_disp;
                        cube.masses[u].position[2] -= z_disp;
                    }
                    
                    cube.center[0] -= x_disp;
                    cube.center[1] -= y_disp;
                    cube.center[2] -= z_disp;
                }
                
                fuse_faces(all_cubes[cube1], cube, cube1, i, masses, springs, cube1_face1, cube2_face2, masses_left, springs_left);
                
                for (int q=0; q<all_cubes.size(); q++){
                    if (all_cubes[q].center[0]-cube.center[0] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                        
                        int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 2)-all_cubes[q].free_faces.begin();
                        int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 4)-cube.free_faces.begin();
                        
                        all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                        cube.free_faces.erase(cube.free_faces.begin()+itr4);
                        
                        fuse_faces(all_cubes[q], cube, q, i, masses, springs, 2, 4, masses_left, springs_left);
                    }
                    else if (all_cubes[q].center[0]-cube.center[0] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                        
                        int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 4)-all_cubes[q].free_faces.begin();
                        int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 2)-cube.free_faces.begin();
                        
                        all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                        cube.free_faces.erase(cube.free_faces.begin()+itr4);
                        
                        fuse_faces(all_cubes[q], cube, q, i, masses, springs, 4, 2, masses_left, springs_left);
                    }
                    else if (all_cubes[q].center[1]-cube.center[1] == 0.5 && all_cubes[q].center[0]-cube.center[0] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                        
                        int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 1)-all_cubes[q].free_faces.begin();
                        int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 3)-cube.free_faces.begin();
                        
                        all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                        cube.free_faces.erase(cube.free_faces.begin()+itr4);
                        
                        fuse_faces(all_cubes[q], cube, q, i, masses, springs, 1, 3, masses_left, springs_left);
                    }
                    else if (all_cubes[q].center[1]-cube.center[1] == -0.5 && all_cubes[q].center[2]-cube.center[2] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                        
                        int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 3)-all_cubes[q].free_faces.begin();
                        int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 1)-cube.free_faces.begin();
                        
                        all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                        cube.free_faces.erase(cube.free_faces.begin()+itr4);
                        
                        fuse_faces(all_cubes[q], cube, q, i, masses, springs, 3, 1, masses_left, springs_left);
                    }
                    else if (all_cubes[q].center[2]-cube.center[2] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                        
                        int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 0)-all_cubes[q].free_faces.begin();
                        int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 5)-cube.free_faces.begin();
                        
                        all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                        cube.free_faces.erase(cube.free_faces.begin()+itr4);
                        
                        fuse_faces(all_cubes[q], cube, q, i, masses, springs, 0, 5, masses_left, springs_left);
                    }
                    else if (all_cubes[q].center[2]-cube.center[2] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                        
                        int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 5)-all_cubes[q].free_faces.begin();
                        int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 0)-cube.free_faces.begin();
                        
                        all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                        cube.free_faces.erase(cube.free_faces.begin()+itr4);
                        
                        fuse_faces(all_cubes[q], cube, q, i, masses, springs, 5, 0, masses_left, springs_left);
                    }
                }
                
                for (int j=0; j<masses_left.size(); j++){
                    // if the vertex is not part of face 2 then you can add it to the big vector of masses and make the ID the index of where it is in the big vector of masses
                    cube.masses[masses_left[j]].ID = masses.size();
                    cube.massIDs.push_back(masses.size());
                    masses.push_back(cube.masses[masses_left[j]]);
                    
                }
                
                for (int k=0; k<springs_left.size(); k++){
                    int p0 = cube.springs[springs_left[k]].m0;
                    int p1 = cube.springs[springs_left[k]].m1;
                    
                    cube.springs[springs_left[k]].m0 = cube.masses[p0].ID;
                    cube.springs[springs_left[k]].m1 = cube.masses[p1].ID;
                    cube.springs[springs_left[k]].ID = springs.size();
                    cube.springIDs.push_back(springs.size());
                    springs.push_back(cube.springs[springs_left[k]]);
                    
                }
                
                if (cube.free_faces.size() < 1){
                    cout << "Maximized fused faces on this cube" << endl;
                }
                else{
                    available_cubes.push_back(i);
                }
                if (all_cubes[cube1].free_faces.size() < 1){
                    cout << "Maximized fused faces on this cube" << endl;
                    int itr5 = find(available_cubes.begin(), available_cubes.end(), cube1)-available_cubes.begin();
                    available_cubes.erase(available_cubes.begin()+itr5);
                }
                
            }
            
        }
        else{
            int cube1 = robot2.all_cubes[i].joinedCubes[0];
            int cube2_face2 = robot2.all_cubes[i].joinedFaces[0];
            int cube1_face1;
            
            vector<int> map1;
            vector<int> map2;
            vector<int> masses_left;
            vector<int> springs_left;
            
            for (int s=0; s<8; s++){
                masses_left.push_back(s);
            }
            
            for (int v=0; v<28; v++){
                springs_left.push_back(v);
            }
            
            if (cube2_face2 == 0){
                cube1_face1 = 5;
                
                map2 = face0;
                map1 = face5;
            }
            else if (cube2_face2 == 5){
                cube1_face1 = 0;
                
                map2 = face5;
                map1 = face0;
            }
            else if (cube2_face2 == 1){
                cube1_face1 = 3;
                
                map2 = face1;
                map1 = face3;
            }
            else if (cube2_face2 == 3){
                cube1_face1 = 1;
                
                map2 = face3;
                map1 = face1;
            }
            else if (cube2_face2 == 2){
                cube1_face1 = 4;
                
                map2 = face2;
                map1 = face4;
            }
            else{
                cube1_face1 = 2;
                
                map2 = face4;
                map1 = face2;
            }
            
            if (find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1) == all_cubes[cube1].free_faces.end()){
                bool clashing = true;
                cout << "CLASHING" << endl;
                while (clashing) {
                    int itr6 = find(all_cubes[cube1].joinedFaces.begin(), all_cubes[cube1].joinedFaces.end(), cube1_face1)-all_cubes[cube1].joinedFaces.begin();
                    cube1 = all_cubes[cube1].joinedCubes[itr6];
                    if (find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1) != all_cubes[cube1].free_faces.end()){
                        clashing = false;
                    }
                }
                cout << "RESOLVED" << endl;
            }
            
            int itr = find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1)-all_cubes[cube1].free_faces.begin();
            int itr2 = find(cube.free_faces.begin(), cube.free_faces.end(), cube2_face2)-cube.free_faces.begin();
            
            all_cubes[cube1].free_faces.erase(all_cubes[cube1].free_faces.begin()+itr);
            cube.free_faces.erase(cube.free_faces.begin()+itr2);
            
            float cube1_z0 = all_cubes[cube1].masses[0].position[2];
            
            if (cube1_face1 == 0 && cube1_z0 == 0){
                float x_disp = all_cubes[cube1].masses[map1[0]].position[0]-cube.masses[map2[0]].position[0]; //x displacement
                float y_disp = all_cubes[cube1].masses[map1[0]].position[1]-cube.masses[map2[0]].position[1]; //y displacement
                float z_disp = all_cubes[cube1].masses[map1[0]].position[2]-cube.masses[map2[0]].position[2]; //z displacement
                
                for (int m=0; m<all_cubes.size(); m++){
                    for (int n=0; n<8; n++){
                        //shift cube 2 over
                        all_cubes[m].masses[n].position[0] -= x_disp;
                        all_cubes[m].masses[n].position[1] -= y_disp;
                        all_cubes[m].masses[n].position[2] -= z_disp;
                        
                        masses[all_cubes[m].masses[n].ID].position[0] = all_cubes[m].masses[n].position[0];
                        masses[all_cubes[m].masses[n].ID].position[1] = all_cubes[m].masses[n].position[1];
                        masses[all_cubes[m].masses[n].ID].position[2] = all_cubes[m].masses[n].position[2];
                    }
                    all_cubes[m].center[0] -= x_disp;
                    all_cubes[m].center[1] -= y_disp;
                    all_cubes[m].center[2] -= z_disp;
                }
            }
            else{
                //find where the second cube needs to join the first cube
                float x_disp = cube.masses[map2[0]].position[0]-all_cubes[cube1].masses[map1[0]].position[0]; //x displacement
                float y_disp = cube.masses[map2[0]].position[1]-all_cubes[cube1].masses[map1[0]].position[1]; //y displacement
                float z_disp = cube.masses[map2[0]].position[2]-all_cubes[cube1].masses[map1[0]].position[2]; //z displacement
                
                for (int u=0; u<8; u++){
                    //shift cube 2 over
                    cube.masses[u].position[0] -= x_disp;
                    cube.masses[u].position[1] -= y_disp;
                    cube.masses[u].position[2] -= z_disp;
                }
                
                cube.center[0] -= x_disp;
                cube.center[1] -= y_disp;
                cube.center[2] -= z_disp;
            }
            
            fuse_faces(all_cubes[cube1], cube, cube1, i, masses, springs, cube1_face1, cube2_face2, masses_left, springs_left);
            
            for (int q=0; q<all_cubes.size(); q++){
                if (all_cubes[q].center[0]-cube.center[0] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 2)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 4)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 2, 4, masses_left, springs_left);
                }
                else if (all_cubes[q].center[0]-cube.center[0] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 4)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 2)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 4, 2, masses_left, springs_left);
                }
                else if (all_cubes[q].center[1]-cube.center[1] == 0.5 && all_cubes[q].center[0]-cube.center[0] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 1)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 3)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 1, 3, masses_left, springs_left);
                }
                else if (all_cubes[q].center[1]-cube.center[1] == -0.5 && all_cubes[q].center[2]-cube.center[2] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 3)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 1)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 3, 1, masses_left, springs_left);
                }
                else if (all_cubes[q].center[2]-cube.center[2] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 0)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 5)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 0, 5, masses_left, springs_left);
                }
                else if (all_cubes[q].center[2]-cube.center[2] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 5)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 0)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 5, 0, masses_left, springs_left);
                }
            }
            
            
            for (int j=0; j<masses_left.size(); j++){
                // if the vertex is not part of face 2 then you can add it to the big vector of masses and make the ID the index of where it is in the big vector of masses
                cube.masses[masses_left[j]].ID = masses.size();
                cube.massIDs.push_back(masses.size());
                masses.push_back(cube.masses[masses_left[j]]);
                
            }
            
            for (int k=0; k<springs_left.size(); k++){
                int p0 = cube.springs[springs_left[k]].m0;
                int p1 = cube.springs[springs_left[k]].m1;
                
                cube.springs[springs_left[k]].m0 = cube.masses[p0].ID;
                cube.springs[springs_left[k]].m1 = cube.masses[p1].ID;
                cube.springs[springs_left[k]].ID = springs.size();
                cube.springIDs.push_back(springs.size());
                springs.push_back(cube.springs[springs_left[k]]);
                
            }
            
            if (cube.free_faces.size() < 1){
                cout << "Maximized fused faces on this cube" << endl;
            }
            else{
                available_cubes.push_back(i);
            }
            if (all_cubes[cube1].free_faces.size() < 1){
                cout << "Maximized fused faces on this cube" << endl;
                int itr5 = find(available_cubes.begin(), available_cubes.end(), cube1)-available_cubes.begin();
                available_cubes.erase(available_cubes.begin()+itr5);
            }
            
        }
        
        cubes.push_back(i);
        all_cubes.push_back(cube);
    }
    
    offspring.masses = masses;
    offspring.springs = springs;
    offspring.all_cubes = all_cubes;
    offspring.available_cubes = available_cubes;
    
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    for (int m=0; m<offspring.masses.size(); m++){
        x_center += offspring.masses[m].position[0];
        y_center += offspring.masses[m].position[1];
        z_center += offspring.masses[m].position[2];
    }
    
    x_center = x_center/offspring.masses.size();
    y_center = y_center/offspring.masses.size();
    z_center = z_center/offspring.masses.size();
    
    offspring.center = {x_center, y_center, z_center};
    
    for (int c=0; c<population.size(); c++){
        population[c].start = offspring.center;
        float f = determine_fitness(population[c], offspring);
        if (f > offspring.fitness){
            offspring.fitness = f;
            offspring.best_controller = population[c];
        }
        if (f > population[c].fitness){
            population[c].fitness = f;
        }
    }
    
    if (major_league.size() > 0){
        for (int m=0; m<major_league.size(); m++){
            major_league[m].start = offspring.center;
            float f = determine_fitness(major_league[m], offspring);
            if (f > offspring.fitness){
                offspring.fitness = f;
                offspring.best_controller = major_league[m];
            }
            if (f > major_league[m].fitness){
                major_league[m].fitness = f;
            }
        }
    }
    
    if (offspring.fitness > robot1.fitness){
        new_robot_population.push_back(offspring);
    }
    else{
        new_robot_population.push_back(robot1);
    }
}
//-----------------------------------------------------------------------

//ROBOT AND CUBE INITIALIZATION HAPPENS HERE AND BELOW
//-----------------------------------------------------------------------
void initialize_robot(Robot &robot){
    vector<PointMass> masses; //initializes the vector of masses that make up the robot
    vector<Spring> springs; //initializes the vector of springs that make up the robot
    vector<int> cubes;
    vector<Cube> all_cubes; //initializes all the cubes that will make up this robot
    vector<int> available_cubes;
    for (int i=0; i<14; i++){
        Cube cube; //define a cube
        initialize_cube(cube); //initialize the cube
        if (i==0){
            //for the first cube, you can add everything
            for (int j=0; j<28; j++){
                cube.springs[j].ID = j;
                cube.springIDs.push_back(j);
                springs.push_back(cube.springs[j]);
            }
            for (int k=0; k<8; k++){
                cube.masses[k].ID = k;
                cube.massIDs.push_back(k);
                masses.push_back(cube.masses[k]);
            }
            available_cubes.push_back(i);
        }
        else{
            int cube1 = rand() % available_cubes.size();
            cube1 = available_cubes[cube1];
            int face_1 = rand() % all_cubes[cube1].free_faces.size();
            int cube1_face1 = all_cubes[cube1].free_faces[face_1];
//            int cube1_face1 = 5;
            int face_2;
            vector<int> map1;
            vector<int> map2;
            vector<int> masses_left;
            vector<int> springs_left;
            
            for (int s=0; s<8; s++){
                masses_left.push_back(s);
            }
            
            for (int v=0; v<28; v++){
                springs_left.push_back(v);
            }
            
            if (cube1_face1 == 0){
                face_2 = 5;
                
                map1 = face0;
                map2 = face5;
            }
            else if (cube1_face1 == 5){
                face_2 = 0;
                
                map1 = face5;
                map2 = face0;
            }
            else if (cube1_face1 == 1){
                face_2 = 3;
                
                map1 = face1;
                map2 = face3;
            }
            else if (cube1_face1 == 3){
                face_2 = 1;
                
                map1 = face3;
                map2 = face1;
            }
            else if (cube1_face1 == 2){
                face_2 = 4;
                
                map1 = face2;
                map2 = face4;
            }
            else{
                face_2 = 2;
                
                map1 = face4;
                map2 = face2;
            }
            
            int itr = find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1)-all_cubes[cube1].free_faces.begin();
            int itr2 = find(cube.free_faces.begin(), cube.free_faces.end(), face_2)-cube.free_faces.begin();
            
            all_cubes[cube1].free_faces.erase(all_cubes[cube1].free_faces.begin()+itr);
            cube.free_faces.erase(cube.free_faces.begin()+itr2);
//            remove(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), all_cubes[cube1].free_faces[face_1]);
//            remove(cube.free_faces.begin(), cube.free_faces.end(), cube.free_faces[face_2]);
            
            float cube1_z0 = all_cubes[cube1].masses[0].position[2];
            
            if (cube1_face1 == 0 && cube1_z0 == 0){
                cout << "Need to shift the robot up" << endl;
                float x_disp = all_cubes[cube1].masses[map1[0]].position[0]-cube.masses[map2[0]].position[0]; //x displacement
                float y_disp = all_cubes[cube1].masses[map1[0]].position[1]-cube.masses[map2[0]].position[1]; //y displacement
                float z_disp = all_cubes[cube1].masses[map1[0]].position[2]-cube.masses[map2[0]].position[2]; //z displacement
                
                for (int m=0; m<all_cubes.size(); m++){
                    for (int n=0; n<8; n++){
                        //shift cube 2 over
                        all_cubes[m].masses[n].position[0] -= x_disp;
                        all_cubes[m].masses[n].position[1] -= y_disp;
                        all_cubes[m].masses[n].position[2] -= z_disp;
                        
                        masses[all_cubes[m].masses[n].ID].position[0] = all_cubes[m].masses[n].position[0];
                        masses[all_cubes[m].masses[n].ID].position[1] = all_cubes[m].masses[n].position[1];
                        masses[all_cubes[m].masses[n].ID].position[2] = all_cubes[m].masses[n].position[2];
                    }
                    all_cubes[m].center[0] -= x_disp;
                    all_cubes[m].center[1] -= y_disp;
                    all_cubes[m].center[2] -= z_disp;
                }
            }
            else{
                //find where the second cube needs to join the first cube
                float x_disp = cube.masses[map2[0]].position[0]-all_cubes[cube1].masses[map1[0]].position[0]; //x displacement
                float y_disp = cube.masses[map2[0]].position[1]-all_cubes[cube1].masses[map1[0]].position[1]; //y displacement
                float z_disp = cube.masses[map2[0]].position[2]-all_cubes[cube1].masses[map1[0]].position[2]; //z displacement
                
                for (int u=0; u<8; u++){
                    //shift cube 2 over
                    cube.masses[u].position[0] -= x_disp;
                    cube.masses[u].position[1] -= y_disp;
                    cube.masses[u].position[2] -= z_disp;
                }
                
                cube.center[0] -= x_disp;
                cube.center[1] -= y_disp;
                cube.center[2] -= z_disp;
            }
            
            fuse_faces(all_cubes[cube1], cube, cube1, i, masses, springs, cube1_face1, face_2, masses_left, springs_left);
            
            for (int q=0; q<all_cubes.size(); q++){
                if (all_cubes[q].center[0]-cube.center[0] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    cout << "Also a cube to the right" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 2)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 4)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 2, 4, masses_left, springs_left);
                }
                else if (all_cubes[q].center[0]-cube.center[0] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    cout << "Also a cube to the left" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 4)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 2)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 4, 2, masses_left, springs_left);
                }
                else if (all_cubes[q].center[1]-cube.center[1] == 0.5 && all_cubes[q].center[0]-cube.center[0] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    cout << "Also a cube in front" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 1)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 3)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 1, 3, masses_left, springs_left);
                }
                else if (all_cubes[q].center[1]-cube.center[1] == -0.5 && all_cubes[q].center[2]-cube.center[2] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    cout << "Also a cube in back" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 3)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 1)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 3, 1, masses_left, springs_left);
                }
                else if (all_cubes[q].center[2]-cube.center[2] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    cout << "Also a cube on top" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 0)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 5)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 0, 5, masses_left, springs_left);
                }
                else if (all_cubes[q].center[2]-cube.center[2] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    cout << "Also a cube on bottom" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 5)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 0)-cube.free_faces.begin();
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 5, 0, masses_left, springs_left);
                }
            }
            
            for (int j=0; j<masses_left.size(); j++){
                // if the vertex is not part of face 2 then you can add it to the big vector of masses and make the ID the index of where it is in the big vector of masses
                cube.masses[masses_left[j]].ID = masses.size();
                cube.massIDs.push_back(masses.size());
                masses.push_back(cube.masses[masses_left[j]]);
                
            }
            
            for (int k=0; k<springs_left.size(); k++){
                int p0 = cube.springs[springs_left[k]].m0;
                int p1 = cube.springs[springs_left[k]].m1;
                
                cube.springs[springs_left[k]].m0 = cube.masses[p0].ID;
                cube.springs[springs_left[k]].m1 = cube.masses[p1].ID;
                cube.springs[springs_left[k]].ID = springs.size();
                cube.springIDs.push_back(springs.size());
                springs.push_back(cube.springs[springs_left[k]]);
                
            }
            
            if (cube.free_faces.size() < 1){
                cout << "Maximized fused faces on this cube" << endl;
            }
            else{
                available_cubes.push_back(i);
            }
            if (all_cubes[cube1].free_faces.size() < 1){
                cout << "Maximized fused faces on this cube" << endl;
                int itr5 = find(available_cubes.begin(), available_cubes.end(), cube1)-available_cubes.begin();
                available_cubes.erase(available_cubes.begin()+itr5);
            }
            
        }
        
        cubes.push_back(i);
        all_cubes.push_back(cube);
    }
    robot.masses = masses;
    robot.springs = springs;
    robot.all_cubes = all_cubes;
    robot.available_cubes = available_cubes;
}

void fuse_faces(Cube &cube1, Cube &cube2, int cube1_index, int cube2_index, vector<PointMass> &masses, vector<Spring> &springs, int combine1, int combine2, vector<int> &masses_left, vector<int> &springs_left){
    
    vector<int> map1;
    vector<int> map2;
    vector<int> map1_springs;
    vector<int> map2_springs;
    
    if (combine1 == 0){
        map1 = face0;
        map2 = face5;
        
        map1_springs = face0_springs;
        map2_springs = face5_springs;
    }
    else if (combine1 == 5){
        map1 = face5;
        map2 = face0;
        
        map1_springs = face5_springs;
        map2_springs = face0_springs;
    }
    else if (combine1 == 1){
        map1 = face1;
        map2 = face3;
        
        map1_springs = face1_springs;
        map2_springs = face3_springs;
    }
    else if (combine1 == 3){
        map1 = face3;
        map2 = face1;
        
        map1_springs = face3_springs;
        map2_springs = face1_springs;
    }
    else if (combine1 == 2){
        map1 = face2;
        map2 = face4;
        
        map1_springs = face2_springs;
        map2_springs = face4_springs;
    }
    else{
        map1 = face4;
        map2 = face2;
        
        map1_springs = face4_springs;
        map2_springs = face2_springs;
    }
    
    cube1.joinedCubes.push_back(cube2_index);
    cube1.joinedFaces.push_back(combine1);
    cube1.otherFaces.push_back(combine2);
    
    cube2.joinedCubes.push_back(cube1_index);
    cube2.joinedFaces.push_back(combine2);
    cube2.otherFaces.push_back(combine1);
    
    //joining cube 2 on the right face of the first cube; this means the left face of cube 2 and the right face of cube 1 will be joined
    for (int j=0; j<map2.size(); j++){
        if (find(cube2.massIDs.begin(), cube2.massIDs.end(), cube1.masses[map1[j]].ID) == cube2.massIDs.end()){
            cube2.masses[map2[j]].ID = cube1.masses[map1[j]].ID; //set the mass ID to its position in the masses vector of the robot
            cube2.massIDs.push_back(cube1.masses[map1[j]].ID); //add the mass IDs to the list of masses that correspond to cube2
        }
        
        if (find(masses_left.begin(), masses_left.end(), map2[j]) != masses_left.end()){
//            remove(masses_left.begin(), masses_left.end(), map2[j]);
            int itr = find(masses_left.begin(), masses_left.end(), map2[j])-masses_left.begin();
            masses_left.erase(masses_left.begin()+itr);
        }
    }
    
    for (int k=0; k<map2_springs.size(); k++){
        int p0 = cube2.springs[map2_springs[k]].m0;
        int p1 = cube2.springs[map2_springs[k]].m1;
        
        cube2.springs[map2_springs[k]].m0 = cube2.masses[p0].ID;
        cube2.springs[map2_springs[k]].m1 = cube2.masses[p1].ID;
        
        if (find(cube2.springIDs.begin(), cube2.springIDs.end(), cube1.springs[map1_springs[k]].ID) == cube2.springIDs.end()){
            cube2.springs[map2_springs[k]].ID = cube1.springs[map1_springs[k]].ID;
            cube2.springIDs.push_back(cube1.springs[map1_springs[k]].ID);
        }
        
        
        if (find(springs_left.begin(), springs_left.end(), map2_springs[k]) != springs_left.end()){
//            remove(springs_left.begin(), springs_left.end(), map2_springs[k]);
            int itr = find(springs_left.begin(), springs_left.end(), map2_springs[k])-springs_left.begin();
            springs_left.erase(springs_left.begin()+itr);
        }
    }
}

void initialize_cube(Cube &cube){
    vector<PointMass> masses;
    vector<Spring> springs;
    
    initialize_masses(masses);
    initialize_springs(springs);
    
    cube.masses = masses;
    cube.springs = springs;
    
    for (int i=0; i<6; i++){
        cube.free_faces.push_back(i);
    }
    
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    for (int m=0; m<cube.masses.size(); m++){
        x_center += cube.masses[m].position[0];
        y_center += cube.masses[m].position[1];
        z_center += cube.masses[m].position[2];
    }
    
    x_center = x_center/cube.masses.size();
    y_center = y_center/cube.masses.size();
    z_center = z_center/cube.masses.size();
    
    cube.center = {x_center, y_center, z_center};
    
}

void initialize_masses(vector<PointMass> &masses){
    //Point Mass of bottom, front left vertex
    //----------------------
    PointMass mass0;
    mass0.mass = 1.0f;
    mass0.position = {-0.25f, -0.25f, 0.0f};
    mass0.velocity = {0.0f, 0.0f, 0.0f};
    mass0.acceleration = {0.0f, 0.0f, 0.0f};
    mass0.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, back left vertex
    //----------------------
    PointMass mass1;
    mass1.mass = 1.0f;
    mass1.position = {-0.25f, 0.25f, 0.0f};
    mass1.velocity = {0.0f, 0.0f, 0.0f};
    mass1.acceleration = {0.0f, 0.0f, 0.0f};
    mass1.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, back right vertex
    //----------------------
    PointMass mass2;
    mass2.mass = 1.0f;
    mass2.position = {0.25f, 0.25f, 0.0f};
    mass2.velocity = {0.0f, 0.0f, 0.0f};
    mass2.acceleration = {0.0f, 0.0f, 0.0f};
    mass2.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, front right vertex
    //----------------------
    PointMass mass3;
    mass3.mass = 1.0f;
    mass3.position = {0.25f, -0.25f, 0.0f};
    mass3.velocity = {0.0f, 0.0f, 0.0f};
    mass3.acceleration = {0.0f, 0.0f, 0.0f};
    mass3.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, front left vertex
    //----------------------
    PointMass mass4;
    mass4.mass = 1.0f;
    mass4.position = {-0.25f, -0.25f, 0.5f};
    mass4.velocity = {0.0f, 0.0f, 0.0f};
    mass4.acceleration = {0.0f, 0.0f, 0.0f};
    mass4.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, back left vertex
    //----------------------
    PointMass mass5;
    mass5.mass = 1.0f;
    mass5.position = {-0.25f, 0.25f, 0.5f};
    mass5.velocity = {0.0f, 0.0f, 0.0f};
    mass5.acceleration = {0.0f, 0.0f, 0.0f};
    mass5.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, back right vertex
    //----------------------
    PointMass mass6;
    mass6.mass = 1.0f;
    mass6.position = {0.25f, 0.25f, 0.5f};
    mass6.velocity = {0.0f, 0.0f, 0.0f};
    mass6.acceleration = {0.0f, 0.0f, 0.0f};
    mass6.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, front right vertex
    //----------------------
    PointMass mass7;
    mass7.mass = 1.0f;
    mass7.position = {0.25f, -0.25f, 0.5f};
    mass7.velocity = {0.0f, 0.0f, 0.0f};
    mass7.acceleration = {0.0f, 0.0f, 0.0f};
    mass7.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    masses = {mass0, mass1, mass2, mass3, mass4, mass5, mass6, mass7};
    
}

void initialize_springs(vector<Spring> &springs){
    
    //Bottom Face of the Cube
    //-----------------------
    Spring spring0;
    spring0.L0 = 0.5f;
    spring0.L = 0.5f;
    spring0.k = spring_constant;
    spring0.m0 = 0;
    spring0.m1 = 1;
    spring0.original_L0 = 0.5f;
    
    Spring spring1;
    spring1.L0 = 0.5f;
    spring1.L = 0.5f;
    spring1.k = spring_constant;
    spring1.m0 = 1;
    spring1.m1 = 2;
    spring1.original_L0 = 0.5f;
    
    Spring spring2;
    spring2.L0 = 0.5f;
    spring2.L = 0.5f;
    spring2.k = spring_constant;
    spring2.m0 = 2;
    spring2.m1 = 3;
    spring2.original_L0 = 0.5f;
    
    Spring spring3;
    spring3.L0 = 0.5f;
    spring3.L = 0.5f;
    spring3.k = spring_constant;
    spring3.m0 = 3;
    spring3.m1 = 0;
    spring3.original_L0 = 0.5f;
    //----------------------
    
    //Cross Springs of Bottom Face
    //----------------------
    Spring spring4;
    spring4.L0 = 0.5f*sqrt(2.0f);
    spring4.L = 0.5f*sqrt(2.0f);
    spring4.k = spring_constant;
    spring4.m0 = 0;
    spring4.m1 = 2;
    spring4.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring5;
    spring5.L0 = 0.5f*sqrt(2.0f);
    spring5.L = 0.5f*sqrt(2.0f);
    spring5.k = spring_constant;
    spring5.m0 = 1;
    spring5.m1 = 3;
    spring5.original_L0 = 0.5f*sqrt(2.0f);
    //----------------------
    
    //Vertical Supports of Cube
    //----------------------
    Spring spring6;
    spring6.L0 = 0.5f;
    spring6.L = 0.5f;
    spring6.k = spring_constant;
    spring6.m0 = 0;
    spring6.m1 = 4;
    spring6.original_L0 = 0.5f;
    
    Spring spring7;
    spring7.L0 = 0.5f;
    spring7.L = 0.5f;
    spring7.k = spring_constant;
    spring7.m0 = 1;
    spring7.m1 = 5;
    spring7.original_L0 = 0.5f;
    
    Spring spring8;
    spring8.L0 = 0.5f;
    spring8.L = 0.5f;
    spring8.k = spring_constant;
    spring8.m0 = 2;
    spring8.m1 = 6;
    spring8.original_L0 = 0.5f;
    
    Spring spring9;
    spring9.L0 = 0.5f;
    spring9.L = 0.5f;
    spring9.k = spring_constant;
    spring9.m0 = 3;
    spring9.m1 = 7;
    spring9.original_L0 = 0.5f;
    //---------------------
    
    //Cross Springs of Front Face
    //---------------------
    Spring spring10;
    spring10.L0 = 0.5f*sqrt(2.0f);
    spring10.L = 0.5f*sqrt(2.0f);
    spring10.k = spring_constant;
    spring10.m0 = 0;
    spring10.m1 = 7;
    spring10.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring11;
    spring11.L0 = 0.5f*sqrt(2.0f);
    spring11.L = 0.5f*sqrt(2.0f);
    spring11.k = spring_constant;
    spring11.m0 = 3;
    spring11.m1 = 4;
    spring11.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Left Face
    //---------------------
    Spring spring12;
    spring12.L0 = 0.5f*sqrt(2.0f);
    spring12.L = 0.5f*sqrt(2.0f);
    spring12.k = spring_constant;
    spring12.m0 = 0;
    spring12.m1 = 5;
    spring12.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring13;
    spring13.L0 = 0.5f*sqrt(2.0f);
    spring13.L = 0.5f*sqrt(2.0f);
    spring13.k = spring_constant;
    spring13.m0 = 1;
    spring13.m1 = 4;
    spring13.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Back Face
    //---------------------
    Spring spring14;
    spring14.L0 = 0.5f*sqrt(2.0f);
    spring14.L = 0.5f*sqrt(2.0f);
    spring14.k = spring_constant;
    spring14.m0 = 1;
    spring14.m1 = 6;
    spring14.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring15;
    spring15.L0 = 0.5f*sqrt(2.0f);
    spring15.L = 0.5f*sqrt(2.0f);
    spring15.k = spring_constant;
    spring15.m0 = 2;
    spring15.m1 = 5;
    spring15.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Right Face
    //---------------------
    Spring spring16;
    spring16.L0 = 0.5f*sqrt(2.0f);
    spring16.L = 0.5f*sqrt(2.0f);
    spring16.k = spring_constant;
    spring16.m0 = 2;
    spring16.m1 = 7;
    spring16.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring17;
    spring17.L0 = 0.5f*sqrt(2.0f);
    spring17.L = 0.5f*sqrt(2.0f);
    spring17.k = spring_constant;
    spring17.m0 = 3;
    spring17.m1 = 6;
    spring17.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Top Face of the Cube
    //---------------------
    Spring spring18;
    spring18.L0 = 0.5f;
    spring18.L = 0.5f;
    spring18.k = spring_constant;
    spring18.m0 = 4;
    spring18.m1 = 5;
    spring18.original_L0 = 0.5f;
    
    Spring spring19;
    spring19.L0 = 0.5f;
    spring19.L = 0.5f;
    spring19.k = spring_constant;
    spring19.m0 = 5;
    spring19.m1 = 6;
    spring19.original_L0 = 0.5f;
    
    Spring spring20;
    spring20.L0 = 0.5f;
    spring20.L = 0.5f;
    spring20.k = spring_constant;
    spring20.m0 = 6;
    spring20.m1 = 7;
    spring20.original_L0 = 0.5f;
    
    Spring spring21;
    spring21.L0 = 0.5f;
    spring21.L = 0.5f;
    spring21.k = spring_constant;
    spring21.m0 = 7;
    spring21.m1 = 4;
    spring21.original_L0 = 0.5f;
    //---------------------
    
    //Cross Springs of Top Face
    //---------------------
    Spring spring22;
    spring22.L0 = 0.5f*sqrt(2.0f);
    spring22.L = 0.5f*sqrt(2.0f);
    spring22.k = spring_constant;
    spring22.m0 = 4;
    spring22.m1 = 6;
    spring22.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring23;
    spring23.L0 = 0.5f*sqrt(2.0f);
    spring23.L = 0.5f*sqrt(2.0f);
    spring23.k = spring_constant;
    spring23.m0 = 5;
    spring23.m1 = 7;
    spring23.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Inner Cross Springs
    //---------------------
    Spring spring24;
    spring24.L0 = 0.5f*sqrt(3.0f);
    spring24.L = 0.5f*sqrt(3.0f);
    spring24.k = spring_constant;
    spring24.m0 = 0;
    spring24.m1 = 6;
    spring24.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring25;
    spring25.L0 = 0.5f*sqrt(3.0f);
    spring25.L = 0.5f*sqrt(3.0f);
    spring25.k = spring_constant;
    spring25.m0 = 2;
    spring25.m1 = 4;
    spring25.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring26;
    spring26.L0 = 0.5f*sqrt(3.0f);
    spring26.L = 0.5f*sqrt(3.0f);
    spring26.k = spring_constant;
    spring26.m0 = 1;
    spring26.m1 = 7;
    spring26.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring27;
    spring27.L0 = 0.5f*sqrt(3.0f);
    spring27.L = 0.5f*sqrt(3.0f);
    spring27.k = spring_constant;
    spring27.m0 = 3;
    spring27.m1 = 5;
    spring27.original_L0 = 0.5f*sqrt(3.0f);
    //---------------------
    
    springs = {spring0, spring1, spring2, spring3, spring4, spring5, spring6, spring7, spring8, spring9, spring10, spring11, spring12, spring13, spring14, spring15, spring16, spring17, spring18, spring19, spring20, spring21, spring22, spring23, spring24, spring25, spring26, spring27};
}
//-----------------------------------------------------------------------

bool compareByFitness(const Controller &control1, const Controller &control2){
    return control1.fitness > control2.fitness;
}

bool compareByFitnessR(const Robot &robot1, const Robot &robot2){
    return robot1.fitness > robot2.fitness;
}
