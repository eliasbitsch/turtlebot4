#include <iostream>
#include <cmath>
#include <iomanip>

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
void init_map(WaypointMap* m){for(int i=0;i<MAX_WPTS;i++)m->used[i]=0;}
int str_eq(const char* a,const char* b){int i=0;while(a[i]&&b[i]){if(a[i]!=b[i])return 0;i++;}return(a[i]==b[i]);}
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
void init_grid(OccupancyGrid &g){g.width=GRID_WIDTH;g.height=GRID_HEIGHT;for(int x=0;x<GRID_WIDTH;x++)for(int y=0;y<GRID_HEIGHT;y++)g.cells[x][y]=CELL_FREE;}
void addObstacle(OccupancyGrid &g,int x,int y){if(x>=0&&x<g.width&&y>=0&&y<g.height)g.cells[x][y]=CELL_OCCUPIED;}

// -------------------- Simple Greedy Path Planning --------------------
bool nextStep(OccupancyGrid &g,int &x,int &y,int goal_x,int goal_y){
    int dx=(goal_x>x)?1:(goal_x<x)?-1:0;
    int dy=(goal_y>y)?1:(goal_y<y)?-1:0;
    if(dx!=0 && g.cells[x+dx][y]==CELL_FREE){x+=dx;return true;}
    if(dy!=0 && g.cells[x][y+dy]==CELL_FREE){y+=dy;return true;}
    if(dx!=0 && dy!=0 && g.cells[x+dx][y+dy]==CELL_FREE){x+=dx;y+=dy;return true;}
    return false;
}

// -------------------- Generate Pose Path --------------------
int generatePosePath(OccupancyGrid &grid, WaypointMap &map, const char* wp_names[], int num_wpts, Pose pose_path[], int max_len){
    int path_len=0;
    Pose current = *get_waypoint(&map, wp_names[0]); // Startwaypoint

    for(int i=1;i<num_wpts;i++){
        Pose* wp = get_waypoint(&map, wp_names[i]);
        if(!wp) continue;

        int gx=(int)current.x, gy=(int)current.y;
        int goal_x=(int)wp->x, goal_y=(int)wp->y;

        while(gx!=goal_x || gy!=goal_y){
            if(!nextStep(grid,gx,gy,goal_x,goal_y)) break; // Hindernis blockiert
            if(path_len>=max_len) return path_len;
            Pose step={ (double)gx, (double)gy, wp->theta };
            pose_path[path_len++] = step;
        }
        current = *wp;
    }
    return path_len;
}

// -------------------- Main --------------------
int main(){
    WaypointMap map; init_map(&map);
    OccupancyGrid grid; init_grid(grid);

    // Hindernisse im Grid setzen (Beispiel)
    for(int i=10;i<20;i++) addObstacle(grid,i,25);
    for(int i=30;i<40;i++) addObstacle(grid,30,i);

    // Waypoints setzen
    Pose p1={5,5,0}; Pose p2={45,45,0};
    add_waypoint(&map,"start",p1);
    add_waypoint(&map,"goal",p2);

    const char* wp_sequence[] = {"start","goal"};

    // Pose-Liste für LinearController vorbereiten
    const int MAX_PATH=1000;
    Pose pose_path[MAX_PATH];
    int path_len = generatePosePath(grid,map,wp_sequence,2,pose_path,MAX_PATH);

    // Ausgabe zur Kontrolle
    std::cout<<std::fixed<<std::setprecision(2);
    for(int i=0;i<path_len;i++){
        std::cout<<"Pose "<<i<<": x="<<pose_path[i].x<<", y="<<pose_path[i].y<<", theta="<<pose_path[i].theta<<"\n";
    }

    std::cout<<"Total poses: "<<path_len<<"\n";
    return 0;
}
