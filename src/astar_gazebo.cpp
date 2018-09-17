#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "ros/ros.h"
#include <sstream>
/* and not not_eq */
#include <iso646.h>
/* add -lm to command line to compile with this header */
#include <math.h>
#include "geometry_msgs/Twist.h"

#define map_size_rows 30
#define map_size_cols 25

char map[map_size_rows][map_size_cols] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

/* description of graph node */
struct stop
{
    double col, row;
    /* array of indexes of routes from this stop to neighbours in array of all routes */
    int *n;
    int n_len;
    double f, g, h;
    int from;
};

int ind[map_size_rows][map_size_cols] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}

};

/* description of route between two nodes */
struct route
{
    /* route has only one direction! */
    int x; /* index of stop in array of all stops of src of this route */
    int y; /* intex of stop in array of all stops od dst of this route */
    double d;
};

int main(int argc, char **argv)
{
    int i, j, k, l, b, found;
    int p_len = 0;
    int *path = NULL;
    int c_len = 0;
    int *closed = NULL;
    int o_len = 1;
    int *open = (int *)calloc(o_len, sizeof(int));
    double min, tempg;
    int s;
    int e;
    int current;
    int s_len = 0;
    struct stop *stops = NULL;
    int r_len = 0;
    struct route *routes = NULL;

    for (i = 1; i < map_size_rows - 1; i++)
    {
        for (j = 1; j < map_size_cols - 1; j++)
        {
            if (!map[i][j])
            {
                ++s_len;
                stops = (struct stop *)realloc(stops, s_len * sizeof(struct stop));
                int t = s_len - 1;
                stops[t].col = j;
                stops[t].row = i;
                stops[t].from = -1;
                stops[t].g = DBL_MAX;
                stops[t].n_len = 0;
                stops[t].n = NULL;
                ind[i][j] = t;
            }
        }
    }

    /* index of start stop */
    s = 0;
    /* index of finish stop */
    e = s_len - 1;

    for (i = 0; i < s_len; i++)
    {
        stops[i].h = sqrt(pow(stops[e].row - stops[i].row, 2) + pow(stops[e].col - stops[i].col, 2));
    }

    for (i = 1; i < map_size_rows - 1; i++)
    {
        for (j = 1; j < map_size_cols - 1; j++)
        {
            if (ind[i][j] >= 0)
            {
                for (k = i - 1; k <= i + 1; k++)
                {
                    for (l = j - 1; l <= j + 1; l++)
                    {
                        if ((k == i) and (l == j))
                        {
                            continue;
                        }
                        if (ind[k][l] >= 0)
                        {
                            ++r_len;
                            routes = (struct route *)realloc(routes, r_len * sizeof(struct route));
                            int t = r_len - 1;
                            routes[t].x = ind[i][j];
                            routes[t].y = ind[k][l];
                            routes[t].d = sqrt(pow(stops[routes[t].y].row - stops[routes[t].x].row, 2) + pow(stops[routes[t].y].col - stops[routes[t].x].col, 2));
                            ++stops[routes[t].x].n_len;
                            stops[routes[t].x].n = (int *)realloc(stops[routes[t].x].n, stops[routes[t].x].n_len * sizeof(int));
                            stops[routes[t].x].n[stops[routes[t].x].n_len - 1] = t;
                        }
                    }
                }
            }
        }
    }

    open[0] = s;
    stops[s].g = 0;
    stops[s].f = stops[s].g + stops[s].h;
    found = 0;

    while (o_len and not found)
    {
        min = DBL_MAX;

        for (i = 0; i < o_len; i++)
        {
            if (stops[open[i]].f < min)
            {
                current = open[i];
                min = stops[open[i]].f;
            }
        }

        if (current == e)
        {
            found = 1;

            ++p_len;
            path = (int *)realloc(path, p_len * sizeof(int));
            path[p_len - 1] = current;
            while (stops[current].from >= 0)
            {
                current = stops[current].from;
                ++p_len;
                path = (int *)realloc(path, p_len * sizeof(int));
                path[p_len - 1] = current;
            }
        }

        for (i = 0; i < o_len; i++)
        {
            if (open[i] == current)
            {
                if (i not_eq (o_len - 1))
                {
                    for (j = i; j < (o_len - 1); j++)
                    {
                        open[j] = open[j + 1];
                    }
                }
                --o_len;
                open = (int *)realloc(open, o_len * sizeof(int));
                break;
            }
        }

        ++c_len;
        closed = (int *)realloc(closed, c_len * sizeof(int));
        closed[c_len - 1] = current;

        for (i = 0; i < stops[current].n_len; i++)
        {
            b = 0;

            for (j = 0; j < c_len; j++)
            {
                if (routes[stops[current].n[i]].y == closed[j])
                {
                    b = 1;
                }
            }

            if (b)
            {
                continue;
            }

            tempg = stops[current].g + routes[stops[current].n[i]].d;

            b = 1;

            if (o_len > 0)
            {
                for (j = 0; j < o_len; j++)
                {
                    if (routes[stops[current].n[i]].y == open[j])
                    {
                        b = 0;
                    }
                }
            }

            if (b or (tempg < stops[routes[stops[current].n[i]].y].g))
            {
                stops[routes[stops[current].n[i]].y].from = current;
                stops[routes[stops[current].n[i]].y].g = tempg;
                stops[routes[stops[current].n[i]].y].f = stops[routes[stops[current].n[i]].y].g + stops[routes[stops[current].n[i]].y].h;

                if (b)
                {
                    ++o_len;
                    open = (int *)realloc(open, o_len * sizeof(int));
                    open[o_len - 1] = routes[stops[current].n[i]].y;
                }
            }
        }
    }

    for (i = 0; i < map_size_rows; i++)
    {
        for (j = 0; j < map_size_cols; j++)
        {
            if (map[i][j])
            {
                putchar(0xdb);
            }
            else
            {
                b = 0;
                for (k = 0; k < p_len; k++)
                {
                    if (ind[i][j] == path[k])
                    {
                        ++b;
                    }
                }
                if (b)
                {
                    putchar('x');
                }
                else
                {
                    putchar('.');
                }
            }
        }
        putchar('\n');
    }

    if (not found)
    {
        puts("IMPOSSIBLE");
    }
    else
    {
        printf("path cost is %d:\n", p_len);
        for (i = p_len - 1; i >= 0; i--)
        {
            printf("(%1.0f, %1.0f)\n", stops[path[i]].row, stops[path[i]].col);
        }
    }

    // for (i = 0; i < s_len; ++i)
    // {
    //     free(stops[i].n);
    // }
    // free(stops);
    // free(routes);
    // free(path);
    // free(open);
    // free(closed);

    ros::init(argc, argv, "a_star_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate loop_rate(10);

    int cnt = p_len - 1; //es 41
    std::cout << cnt << std::endl;
    int col = 0, row = 0;

    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

    while (ros::ok())
    {
        if (cnt < 0)
        {
            break;
        }

        col = stops[path[cnt - 1]].col - stops[path[cnt]].col;
        row = stops[path[cnt - 1]].row - stops[path[cnt]].row;
        std::cout << row << " y " << col << "cnt es:" << cnt << std::endl;

        if (row == 1 && col == 0)
        {

            msg.linear.x = 0.505;
            msg.angular.z = 0;
        }
        else if (row == 1 && col == 1)
        {
            
            msg.linear.x = 0;
            msg.angular.z = 0.498;
            pub.publish(msg);
            sleep(3);
            msg.linear.x = 0.636;
            msg.angular.z = 0;
            pub.publish(msg);
            sleep(3);
            msg.linear.x = 0;
            msg.angular.z = -0.498;
            pub.publish(msg);
        }
        else if (row == 0 && col == 1)
        {
            
            msg.linear.x = 0;
            msg.angular.z = -0.498;
            pub.publish(msg);
            sleep(3);
            msg.linear.x = 0;
            msg.angular.z = -0.498;;
            pub.publish(msg);
            sleep(3);
            msg.linear.x = 0.505;
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = 0;
            msg.angular.z = 0.498;
            pub.publish(msg);
            sleep(3);
            msg.linear.x = 0;
            msg.angular.z = 0.498;;
            pub.publish(msg);
            sleep(3);
        }
        else
        {
            msg.linear.x = 0;
            msg.angular.z = 0;
        }
        // else if(col==1 && row==1 ){
        // msg.linear.x = 0;
        // msg.angular.z = 0.498;
        // }
        // else if(col==-1 && row==0 ){
        // msg.linear.x = -0.505;
        // msg.angular.z = 0;
        // }

        // else
        // {
        // msg.linear.x = 0;
        // msg.angular.z = 0;
        // }

        cnt--;
        pub.publish(msg);
        //ros::spinOnce();
        //loop_rate.sleep();
        // sleep(3);
        // msg.linear.x = 0;
        // msg.angular.z = 0;

        // pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        sleep(3);
    }

    return 0;
}