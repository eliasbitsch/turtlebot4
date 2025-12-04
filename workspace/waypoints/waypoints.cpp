#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <iomanip>

// -------------------- LinearController --------------------
class LinearController {
public:
    LinearController(double kp_linear=3.0, double k_alpha=8.0, double k_beta=-1.5,
                     double pos_threshold=0.1, double ang_threshold=0.1)
        : kp_linear_(kp_linear), k_alpha_(k_alpha), k_beta_(k_beta),
          POSITION_THRESHOLD(pos_threshold), ANGLE_THRESHOLD(ang_threshold) {}

    void computeControl(double current_x, double current_y, double current_theta,
                        double goal_x, double goal_y, double goal_theta,
                        double &out_linear, double &out_angular) {
        double dx = goal_x - current_x;
        double dy = goal_y - current_y;
        double distance = std::sqrt(dx*dx + dy*dy);

        double angle_to_goal = std::atan2(dy, dx);
        double alpha = normalizeAngle(angle_to_goal - current_theta);
        double beta = normalizeAngle(goal_theta - current_theta - alpha);

        out_linear = 0.0;
        out_angular = 0.0;

        if (distance < POSITION_THRESHOLD && std::fabs(beta) < ANGLE_THRESHOLD) return;

        if (distance >= POSITION_THRESHOLD) {
            out_linear = kp_linear_ * distance * std::cos(alpha);
            out_angular = (k_alpha_ * alpha) + (k_beta_ * beta);
        } else {
            out_linear = 0.0;
            out_angular = k_beta_ * beta;
        }

        const double MAX_LINEAR = 0.22;
        const double MAX_ANGULAR = 2.84;
        if (out_linear > MAX_LINEAR) out_linear = MAX_LINEAR;
        if (out_linear < -MAX_LINEAR) out_linear = -MAX_LINEAR;
        if (out_angular > MAX_ANGULAR) out_angular = MAX_ANGULAR;
        if (out_angular < -MAX_ANGULAR) out_angular = -MAX_ANGULAR;
    }

    static double normalizeAngle(double a) {
        while (a > M_PI) a -= 2.0*M_PI;
        while (a < -M_PI) a += 2.0*M_PI;
        return a;
    }

private:
    double kp_linear_, k_alpha_, k_beta_;
    const double POSITION_THRESHOLD, ANGLE_THRESHOLD;
};

// -------------------- Pose --------------------
struct Pose {
    double x;
    double y;
    double theta;
};

// -------------------- Waypoint Map --------------------
#define MAX_WPTS 32
struct WaypointMap {
    char names[MAX_WPTS][32];
    Pose poses[MAX_WPTS];
    int used[MAX_WPTS];
};

void init_map(WaypointMap* m) { for(int i=0;i<MAX_WPTS;i++) m->used[i]=0; }
int str_eq(const char* a, const char* b){int i=0;while(a[i]&&b[i]){if(a[i]!=b[i])return 0;i++;}return(a[i]==b[i]);}
void str_copy(char* dst,const char* src){int i=0;while(src[i]){dst[i]=src[i];i++;}dst[i]=0;}
int add_waypoint(WaypointMap* m,const char* name,Pose p){for(int i=0;i<MAX_WPTS;i++){if(!m->used[i]){m->used[i]=1;str_copy(m->names[i],name);m->poses[i]=p;return 1;}}return 0;}
Pose* get_waypoint(WaypointMap* m,const char* name){for(int i=0;i<MAX_WPTS;i++){if(m->used[i]&&str_eq(m->names[i],name))return &m->poses[i];}return 0;}

// -------------------- Occupancy Grid --------------------
#define GRID_WIDTH 50
#define GRID_HEIGHT 50
#define CELL_FREE 0
#define CELL_OCCUPIED 1
struct OccupancyGrid {
    int width, height;
    int cells[GRID_WIDTH][GRID_HEIGHT];
};
void init_grid(OccupancyGrid& g){g.width=GRID_WIDTH;g.height=GRID_HEIGHT;for(int x=0;x<GRID_WIDTH;x++)for(int y=0;y<GRID_HEIGHT;y++)g.cells[x][y]=CELL_FREE;}
void addObstacle(OccupancyGrid& g,int x,int y){if(x>=0&&x<g.width&&y>=0&&y<g.height)g.cells[x][y]=CELL_OCCUPIED;}

// -------------------- Simple Greedy Path Planning --------------------
bool nextStep(OccupancyGrid& g,int& x,int& y,int goal_x,int goal_y){
    int dx=(goal_x>x)?1:(goal_x<x)?-1:0;
    int dy=(goal_y>y)?1:(goal_y<y)?-1:0;
    if(dx!=0 && g.cells[x+dx][y]!=CELL_OCCUPIED){x+=dx;return true;}
    if(dy!=0 && g.cells[x][y+dy]!=CELL_OCCUPIED){y+=dy;return true;}
    if(dx!=0 && dy!=0 && g.cells[x+dx][y+dy]!=CELL_OCCUPIED){x+=dx;y+=dy;return true;}
    return false;
}

// -------------------- Navigation --------------------
void navigateToWaypoint(OccupancyGrid &grid, LinearController &controller, Pose &current, Pose &goal){
    int grid_x=(int)current.x, grid_y=(int)current.y;
    int goal_x=(int)goal.x, goal_y=(int)goal.y;
    const double dt=0.1;

    for(int step=0;step<1000;step++){
        if(!nextStep(grid,grid_x,grid_y,goal_x,goal_y)){
            std::cout<<"Obstacle blocking path at "+std::to_string(grid_x)+","+std::to_string(grid_y)+"\n";
            break;
        }
        double v=0.0,w=0.0;
        controller.computeControl(current.x,current.y,current.theta,grid_x,grid_y,goal.theta,v,w);
        current.x+=v*std::cos(current.theta)*dt;
        current.y+=v*std::sin(current.theta)*dt;
        current.theta+=w*dt;
        current.theta=LinearController::normalizeAngle(current.theta);
        double dist=std::sqrt((goal.x-current.x)*(goal.x-current.x)+(goal.y-current.y)*(goal.y-current.y));
        if(dist<0.1)break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// -------------------- Main --------------------
int main(){
    WaypointMap map; init_map(&map);
    OccupancyGrid grid; init_grid(grid);

    // Hindernisse setzen
    for(int i=10;i<20;i++) addObstacle(grid,i,25);
    for(int i=30;i<40;i++) addObstacle(grid,30,i);

    // Waypoints
    Pose p1={5,5,0}; Pose p2={45,45,0};
    add_waypoint(&map,"start",p1);
    add_waypoint(&map,"goal",p2);

    LinearController controller;
    Pose current=p1;

    // Navigation durch Waypoints
    Pose* wp=get_waypoint(&map,"goal");
    if(wp) navigateToWaypoint(grid,controller,current,*wp);

    std::cout<<"Final pose: ("<<current.x<<", "<<current.y<<", "<<current.theta<<")\n";
    return 0;
}