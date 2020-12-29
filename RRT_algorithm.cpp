#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
#include<random>
#include<vector>
#include <unistd.h>
//#include <cstdlib>


using namespace std;
using namespace cv;

class RRT_Visual //CLASS FOR VISUAL REPRESENTATION
{
    public:
    Mat img1;
    int obst;
    int rad = 5;
    Scalar black = Scalar(0,0,0);
    Scalar blue = Scalar(255,0,0);
    Scalar green = Scalar(0,255,0);
    Scalar red = Scalar(0,0,255);
    Scalar white = Scalar(255,255,255);
    Point start1;
    Point goal1;

    RRT_Visual(Point start,Point goal, Mat img, int obstacles)
    {
        img1 = img;
        obst = obstacles;
        start1 = start;
        goal1 = goal;
    }

    void update_map()
    {
        imshow("test",img1);
        waitKey(1);
        
        
    }

    void update1_map()
    {
        imshow("test",img1);
        waitKey(0);
        
        
    }

    void show_img()   //FOR SHOWING STARING AND GOAL NODE
    {
        
        draw_node(start1,'s');
        draw_node(goal1,'g');
        namedWindow("test",CV_WINDOW_AUTOSIZE);
        destroyWindow("test");
    }
    void draw_node(Point pt, char Type)  //FOR DRAWING NODES
    {
        if(Type=='s')
        {
            circle(img1,pt,rad,red,CV_FILLED,LINE_8);
        }
        if(Type == 'g')
        {
            circle(img1,pt,rad,green,CV_FILLED,LINE_8);
        }
        if(Type=='b')
        {
            circle(img1,pt,1,black,CV_FILLED,LINE_8);
        }

    }
    void draw_line(Point node1,Point node2) //FOR DRAWING LINES BETWEEN NODES
    {
        line(img1,node1,node2,black,1.5,LINE_8);
    }
    void draw_line1(Point node1,Point node2) //FOR DRAWING LINES BETWEEN NODES
    {
        line(img1,node1,node2,blue,5,LINE_8);
    }
    void draw_obstacles(vector<Point>obstacles1) //FOR DRAWING OBSTACLES 
    {
        int rec_x;
        int rec_y;
        for(int i=0;i<obst;i++)
        {
            rec_x = obstacles1[i].x;
            rec_y = obstacles1[i].y;
            rectangle(img1,Point(rec_x,rec_y),Point(rec_x+50,rec_y+50),Scalar(0,0,0),CV_FILLED,LINE_8);
        }
    }
    

};

class RRT_Calculation //CALCULATION PART OF ALGORITHM
{
    public:
    Mat img1;
    Point start;
    Point goal;
    bool goal_reached = false;
    vector<Point>nodes;
    vector<int>parents;
    vector<Point>obstacles;
    int obstacle_num;
    int goal_close_point;
    //vector<int>path;
    RRT_Calculation(Point start1, Point goal1, Mat img, int obstacle_number) 
    {
        start = start1;
        goal = goal1;
        img1 = img;
        obstacle_num = obstacle_number;

        nodes.push_back(start);
        parents.push_back(0);

    }



    void create_obstacles(int obstacle_num) //CREATING RANDOM OBSTACLES
    {
        random_device rd;
        mt19937 mt(rd());
        uniform_real_distribution<double> dist(100, 600);
        int x;
        int y;
        
        
        for(int i=0;i<obstacle_num;i++)
        {
            x = dist(mt);
            y = dist(mt);
            Point pt = Point(x,y);
            obstacles.push_back(pt);
            
            //rectangle(img1,Point(x,y),Point(x+50,y+50),Scalar(0,0,0),CV_FILLED,LINE_8);
        }
        //return obstacles;
    }

    vector<Point> return_obstacles()
    {
        create_obstacles(obstacle_num);
        return obstacles;
    }

    Point create_random_point() //CREATING RANDOM POINTS
    {
        random_device rd;
        mt19937 mt(rd());
        uniform_real_distribution<double> dist(0, 750);
        int x,y;
        x = dist(mt);
        y = dist(mt);
        Point pt = Point(x,y);
        return pt;
    }

    void add_node(Point pt,int n)
    {
        auto it = nodes.begin()+n;
        nodes.insert(it,pt);
        //nodes.push_back(pt);
        //cout<<nodes.size()<<endl;
    }

    void remove_node(int n)
    {
         auto it = nodes.begin() + n;
        nodes.erase(it);
        //nodes.pop_back();
    }
    void add_parent(int index, int rand)
    {
        auto it = parents.begin()+rand;
        //parents.insert(it,index);
        parents.push_back(index);
    }
    void remove_parent(int n)
    {
        auto it = parents.begin()+n;
        parents.erase(it);
        //parents.pop_back();

    }

    int number_of_nodes()
    {
        return nodes.size();
    }

    bool free_point(int n) //CHECKING IF ANY NODE IS ON ANY OBSTACLE
    {
        //int n = nodes.size()-1;
        int pt_x = nodes[n].x;
        int pt_y = nodes[n].y;
        // int pt_x = pt.x;
        // int pt_y = pt.y;

        int upper_x;
        int upper_y;
        for(int i =0;i<obstacle_num;i++)
        {
             upper_x = obstacles[i].x;
             upper_y = obstacles[i].y;
             //cout<<pt_x<<" "<<pt_y<<endl;
            if(((upper_x < pt_x) && (pt_x < upper_x+50)) && ((upper_y < pt_y) && (pt_y < upper_y+50)))
            { 
                //cout<<"Executed!"<<endl;

               remove_node(n);   
               return false; 
            }
    }
    return true;
    }

    bool obstacle_overlap(double x1, double x2, double y1, double y2) //CHECKING IF THE LINE BETWEEN ANY TWO NODES DOES NOT OVERLAP WITH OBSTACLES
    {
        double ratio,x,y;
        int upper_x,upper_y;
        for(int i=0;i<obstacle_num;i++)
        {
            upper_x = obstacles[i].x;
            upper_y = obstacles[i].y;
            for(int j=0;j<201;j++)
            {
                ratio = j/200;
                x = x1 * ratio + x2 * (1-ratio);
                y = y1 * ratio + y2 * (1-ratio);
                if(((upper_x < x) && (x < upper_x+50)) && ((upper_y < y) && (y < upper_y+50)))
                {
                    return true;
                }


            }
        }
        return false; 

    }

    void connect_nodes(int n1, int n2) //CONNECTING TWO NODES
    {
        int x1,y1,x2,y2;
        x1 = nodes[n1].x;
        y1 = nodes[n1].y;
        x2 = nodes[n2].x;
        y2 = nodes[n2].y;
        if(obstacle_overlap(x1,x2,y1,y2))
        {
            remove_node(n2);
            //return false;
        }
        else
        {
            add_parent(n1,n2);
            //return true;
        }
        
    }

    double distance(int n1, int n2) //CHECKING DISTANCE BETWEEN TWO NODES
    {
        double x1 = nodes[n1].x;
        double y1 = nodes[n1].y;
        double x2 = nodes[n2].x;
        double y2 = nodes[n2].y;
        double x,y;
        double dist;
        x = pow((x2-x1),2);
        y = pow((y2-y1),2);
        dist = sqrt(x+y);
        
        
        return dist;
    }

    int nearest_node(int n) //FINDING OUT THE NEAREST NODE TO THE RANDOM NODE
    {
        
        int d_min;
        vector<int>distances;
        int nearest;
        for(int i=0;i<n;i++)
        {
            d_min = distance(i,n);
            distances.push_back(d_min);
        }
        nearest = min_element(distances.begin(),distances.end())-distances.begin();
        return nearest;

    }

    void minimum_distance(int n1, int n2)  //SAMPLING THE NODE TO THE MINIMUM DISTANCE
    {
        int d_max = 30;
        double d = distance(n1, n2);
        double x_near,y_near,x_rand,y_rand,p_x,p_y,theta,x,y;
        Point pt;
        if(d>d_max)
        {
            x_near = nodes[n1].x;
            y_near = nodes[n1].y;
            x_rand = nodes[n2].x;
            y_rand = nodes[n2].y;
            p_x = x_rand-x_near;
            p_y = y_rand-y_near;
            theta = atan2(p_y,p_x);
            x = x_near + d_max*cos(theta);
            y = y_near + d_max*sin(theta);
            pt = Point(x,y);
            remove_node(n2);
            add_node(pt, n2);
            if(abs(x-goal.x)<50 && abs(y-goal.y)<50)
            {
                goal_reached = true;
                goal_close_point = n2;
                cout<<"Goal Reached"<<endl;
            }
        }
    }

    vector<int> parents_to_child()
    {
        
        vector<int>path;
        vector<Point>real_path;
        Point pt;
        int index;
        
        int new_position;
        path.push_back(goal_close_point);
        new_position = parents[goal_close_point];
        pt = Point(nodes[new_position].x,nodes[new_position].y);
        
        while(new_position != 0)
        {
            
            

            path.push_back(new_position);
            //cout<<path.back()<<endl;
            new_position = parents[new_position];
        }
        
        path.push_back(0);
        return path; 
    }

    vector<Point> return_nodes()
    {
        return nodes;
    }
    vector<int> return_parents()
    {
        return parents;
    }
    void expand() //METHOD FOR EXPANDING THE TREE   
    {
        int n = nodes.size();
        int near;
        Point pt = create_random_point();
        //cout<<"original point"<<pt.x<<" "<<pt.y<<endl;
        add_node(pt,n);
        if(free_point(n))
        {
            near = nearest_node(n);
            //cout<<"Expand :"<<near<<endl;
            minimum_distance(near,n);
            connect_nodes(near,n);
        }

    }

};

int main()
{
    Point start = Point(20,20);
    Point goal = Point(720,720);
    Point random_point;
    Mat img2(750, 750, CV_8UC3, Scalar(255,255, 255));
    int num_obs;
    cout<<"Enter the number of obstacles"<<endl;
    cin>>num_obs;

    if(num_obs == 0)
    {
        num_obs = 1;
    }
    
    RRT_Visual visual(start,goal,img2,num_obs);
    RRT_Calculation calc(start, goal, img2,num_obs);
    vector<Point>obstacles = calc.return_obstacles();
    visual.draw_obstacles(obstacles);
    visual.show_img();
    vector<Point>nodes1;
    vector<int>parents1;
    vector<int>way;

     //int i=0;
    // int near;
    while(calc.goal_reached==false)
    {
        // cout<<calc.nodes.back().x<<" "<<calc.nodes.back().y<<endl;
        // cout<<"Final Parents: "<<calc.parents.back()<<endl;
        calc.expand();
        nodes1 = calc.return_nodes();
        parents1 = calc.return_parents();

        visual.draw_node(nodes1.back(),'b');
        visual.draw_line(nodes1.back(),nodes1[parents1.back()]);
        visual.update_map();
        //cout<<calc.nodes.size()<<endl;
        //sleep(1);
        //TESTING
        if(calc.goal_reached == true)
        {
            Point pt2;
            Point pt3;
            way = calc.parents_to_child();
            int pt_x,pt_y,pt_x1,pt_y1;
            visual.draw_line1(goal,calc.nodes[way[way.back()]]);
            for(int i =0; i<way.size()-1;i++)
            {
                pt_x = calc.nodes[way[i]].x;
                pt_y = calc.nodes[way[i]].y;
                pt_x1 = calc.nodes[way[i+1]].x;
                pt_y1 = calc.nodes[way[i+1]].y;
                //cout<<"executed"<<pt_x<<" "<<pt_y<<endl;
                pt2 = Point(pt_x,pt_y);
                pt3 = Point(pt_x1,pt_y1);
                visual.draw_line1(pt2,pt3);
                //visual.draw_node(pt2,'s');
                visual.update_map();
                //sleep(1);


            }
            
        }
        //TESTING
        
       
    }
    visual.update1_map();  
    return 0;

}
