#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
//#include "ros/ros.h"
#include <sstream>
/* and not not_eq */
#include <iso646.h>
#include "pioneer.h"
/* add -lm to command line to compile with this header */
#include <math.h>
#include "std_msgs/Float64.h"
//#include "geometry_msgs/Twist.h"

#define map_size_rows 15 //30
#define map_size_cols 18 //25

void move_pioneer(int argc, char **argv, int len, int col[50], int row[50]);
char map[map_size_rows][map_size_cols] = {
    /*{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
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
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};*/

    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

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
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}

};

/* description of route between two nodes */
struct route
{
    /* route has only one direction! */
    int x; /* index of stop in array of all stops of src of this route */
    int y; /* intex of stop in array of all stops od dst of this route */
    double d;
};

float x_vector[200];
int cnt_vector = 0;
int longitud_vector = 0;
int flag_cero_subscriptor = 0;
bool should_continue = 1;
void chatterCallback(const std_msgs::Float64::ConstPtr &msgs)
{
    ROS_INFO("I heard: [%f]", msgs->data);
    if (msgs->data == 0 && flag_cero_subscriptor == 0)
    {
        //do nothing
    }
    else
    {
        flag_cero_subscriptor = 1;
        if (cnt_vector == 0)
        {
            longitud_vector = msgs->data;
            printf("la longitud del vector es %d\n", longitud_vector);
        }
        else
        {
            x_vector[cnt_vector - 1] = msgs->data;
        }

        cnt_vector++;
        if (cnt_vector == longitud_vector + 1)
        { //el +3 es porque los primeros mensajes no los consigue leer de phyton

            std::cout << "quiro salir" << std::endl;
            should_continue = 0;
            //ros::shutdown();
            //std::cout << sub.getTopic() << std::endl;
            //sub.shutdown();
        }
    }
}

int main(int argc, char **argv)
{

    printf("\n");
    printf("//////////////////////////////////////////////////\n");
    printf("\n");
    printf("               PIONNER PATH PLANNING              \n");
    printf("\n");
    printf("\n");
    printf("                       GRVC                       \n");
    printf("//////////////////////////////////////////////////\n\n");

    ros::init(argc, argv, "a_star_publisher");

    char path_key;
    printf("What do you want to use: [A*:a , RRT:r, RRT-A*:t]\n");
    path_key = getchar();
    while (path_key != 'a' && path_key != 'r' && path_key != 't')
    {
        printf("Incorrect path planning, repeat key\n");
        path_key = getchar();
    }

    if (path_key == 'a')
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
        int cnt = p_len - 1;
        int col[cnt], row[cnt];
        while (cnt >= 0)
        {
            col[cnt] = stops[path[cnt - 1]].col - stops[path[cnt]].col;
            row[cnt] = stops[path[cnt - 1]].row - stops[path[cnt]].row;
            std::cout << "col[" << cnt << "]:" << col[cnt] << std::endl;
            std::cout << "row[" << cnt << "]:" << row[cnt] << std::endl;
            cnt--;
        }
        move_pioneer(argc, argv, p_len, col, row);
    }
    else if (path_key == 'r' || path_key == 't')
    {

        int h;
        std::cout << "Waiting to the python program to send us the vector position" << std::endl;
        ros::NodeHandle m;
        ros::Subscriber sub = m.subscribe("python_talker", 1000, chatterCallback);
        //ros::spin();

        ros::Rate r(10); // 10 hz
        while (should_continue)
        {
            ros::spinOnce();
            r.sleep();
        }

        for (h = 0; h <= longitud_vector - 1; h++)
        {
            printf("v:%f\n", x_vector[h]);
        }

        double dec[longitud_vector + 1];
        int int_dec[longitud_vector + 1];
        int i;
        for (i = 0; i <= longitud_vector - 1; i++)
        {
            int_dec[i] = x_vector[i];          //SACAR EL NUMERO ENTERO DEL FLOTANTE
            dec[i] = x_vector[i] - int_dec[i]; //SACAR LA PARTE DECIMAL DEL FLOTANTE
            //printf("parte entera: %d\n", int_dec);
            //printf("parte decimal: %lf\n\n", dec);

            if (x_vector[i] >= 0)
            {
                if ((1 - dec[i]) >= 0.5)
                {
                    int_dec[i] = int_dec[i];
                }
                else
                {
                    int_dec[i] = int_dec[i] + 1;
                }
            }
            else if (x_vector[i] < 0)
            {
                if ((1 + dec[i]) >= 0.5)
                {
                    int_dec[i] = int_dec[i];
                }
                else
                {
                    int_dec[i] = int_dec[i] - 1;
                }
            }
            //printf("int_dec=%d\n", int_dec[i]);
        }
        int col_v[100], row_v[100], col_vv[100], row_vv[100];
        printf("Quito decimales y redondeo\n");
        for (i = 0; i < longitud_vector / 2; i++)
        {
            col_v[i] = int_dec[i];
            printf("col[%d]:%d\n", i, col_v[i]);
        }
        for (i = (longitud_vector / 2); i < longitud_vector; i++)
        {
            row_v[i - (longitud_vector / 2)] = int_dec[i];
            printf("row[%d]:%d\n", i - (longitud_vector / 2), row_v[i - (longitud_vector / 2)]);
        }

        //normalizamos para estar en el mismo sistema de referencia que el a*
        int new_len = (longitud_vector / 2) - 1;
        printf("Cambio de sistema de referencia\n");
        for (i = 0; i <= new_len; i++)
        {
            col_v[i] = -1 * col_v[i] + 1;
            row_v[i] = row_v[i] + 1;
            printf("cccol[%d]:%d\n", i, col_v[i]);
            printf("rrrow[%d]:%d\n", i, row_v[i]);
        }

        //hacemos la resta en la parte del RRT solo
        if (path_key == 'r')
        {
            printf("Hago la resta\n");
            for (i = new_len; i >= 0; i--)
            {

                col_vv[i] = col_v[i - 1] - col_v[i];
                row_vv[i] = row_v[i - 1] - row_v[i];
                printf("col[%d]:%d\n", i, col_vv[i]);
                printf("row[%d]:%d\n", i, row_vv[i]);
            }
        }
        else if (path_key == 't')
        {

            int aux, aux2; //aux2 lo uso para la cuenta
            int cnt_vector_col = 0, cnt_vector_row = 0;
            int flag_col = 0, flag_row = 0;
            col_vv[i] = col_v[i];
            row_vv[i] = row_v[i];
            printf("Hacemos el vector de posiciones mas largooo\n");
            for (i = 0; i <= new_len; i++)
            {
                col_vv[i + cnt_vector_col] = col_v[i];
                row_vv[i + cnt_vector_row] = row_v[i];
                printf("ANTES DE NADA col_VV[%d]:%d\n", i + cnt_vector_col, col_vv[i + cnt_vector_col]);
                printf("ANTES DE NADA row_VV[%d]:%d\n", i + cnt_vector_row, row_vv[i + cnt_vector_row]);
                if ((col_v[i] - col_v[i + 1]) > 1)
                {
                    for (aux = (col_v[i] - col_v[i + 1]) - 1, aux2 = 1; aux >= 1; aux--, aux2++)
                    {
                        cnt_vector_col++;
                        col_vv[i + cnt_vector_col] = col_v[i] - aux2;
                        flag_col++;
                        printf("ENTRAMOS EN COL col_vv[%d]:%d\n", i+cnt_vector_col, col_vv[i+ cnt_vector_col]);
                
                    }
                }

                if ((col_v[i] - col_v[i + 1]) < -1)
                {
                    for (aux = ((col_v[i] - col_v[i + 1]) + 1)*(-1), aux2 = 1; aux >= 1; aux--, aux2++)
                    {
                        cnt_vector_col++;
                        col_vv[i + cnt_vector_col] = col_v[i] + aux2;
                        flag_col++;
                        printf("ENTRAMOS EN COL col_vv[%d]:%d\n", i+cnt_vector_col, col_vv[i+ cnt_vector_col]);
                
                    }
                }

                if ((row_v[i] - row_v[i + 1]) > 1)
                {
                    for (aux = (row_v[i] - row_v[i + 1]) - 1, aux2 = 1; aux >= 1; aux--, aux2++)
                    {
                        cnt_vector_row++;
                        row_vv[i + cnt_vector_row] = row_v[i] - aux2;
                        flag_row++;
                        printf("ENTRAMOS EN row row_vv[%d]:%d\n", i+cnt_vector_row, row_vv[i+ cnt_vector_row]);
                    }
                }

                if ((row_v[i] - row_v[i + 1]) < -1)
                {
                    for (aux = ((row_v[i] - row_v[i + 1]) + 1) * (-1), aux2 = 1; aux >= 1; aux--, aux2++)
                    {
                        cnt_vector_row++;
                        row_vv[i + cnt_vector_row] = row_v[i] + aux2;
                        flag_row++;
                        printf("ENTRAMOS EN row row_vv[%d]:%d\n", i+cnt_vector_row, row_vv[i+ cnt_vector_row]);
                    }
                }


                //printf("2LOC_VV[0]:%d\n", col_vv[0]);
                printf("FLAG COL:%d\n", flag_col);
                printf("FLAG row:%d\n", flag_row);
                while (flag_col !=0 ||  flag_row != 0)
                {

                    if (flag_col >= 1 && flag_row >= 1)
                    {
                        flag_col--;
                        flag_row--;
                        // cnt_vector_row++;
                        // cnt_vector_col++;
                        // col_vv[i + cnt_vector_col] = col_v[i];
                        // row_vv[i + cnt_vector_row] = row_v[i];
                        printf("ENTRAMOS EN fag varios\n");
                        //printf("3LOC_VV[0]:%d\n", col_vv[0]);
                    }

                    else if (flag_col >= 1 && flag_row == 0)
                    {
                        cnt_vector_row++;
                        row_vv[i + cnt_vector_row] = row_v[i+1];
                        flag_col--;
                        //printf("4LOC_VV[0]:%d\n", col_vv[0]);
                        printf("ENTRAMOS EN flag col col_vv[%d]:%d, row_vv[%d]:%d\n", i+cnt_vector_col, col_vv[i+ cnt_vector_col], i + cnt_vector_row,row_vv[i + cnt_vector_row]);
                    }

                    else if (flag_col == 0 && flag_row >= 1)
                    {
                        cnt_vector_col++;
                        col_vv[i + cnt_vector_col] = col_v[i+1];
                        flag_row--;
                        //printf("5LOC_VV[0]:%d\n", col_vv[0]);
                        printf("ENTRAMOS EN flag row  row_vv[%d]:%d, col_vv[%d]:%d\n", i+cnt_vector_row, row_vv[i+ cnt_vector_row],i+cnt_vector_col, col_vv[i+ cnt_vector_col]);
                    }
                }
                printf("FINAL col_VV[%d]:%d\n", i + cnt_vector_col, col_vv[i + cnt_vector_col]);
                printf("FINAL row_VV[%d]:%d\n", i + cnt_vector_row, row_vv[i + cnt_vector_row]);
                //printf("WOR_VV[0]:%d\n", row_vv[0]);
                //printf("6LOC_VV[0]:%d\n", col_vv[0]);
            }
            printf("vectors alargados\n");
            for (i = 0; i <= new_len + cnt_vector_col; i++)
            {
                printf("col[%d]:%d\n", i, col_vv[i]);
                //   printf("row[%d]:%d\n", i, row_vv[i]);
            }
            for (i = 0; i <= new_len + cnt_vector_row; i++)
            {
                printf("row[%d]:%d\n", i, row_vv[i]);
            }
            printf("ponemos los vectores dle mismo tamaño\n");
        }

        move_pioneer(argc, argv, (longitud_vector / 2), col_vv, row_vv);
    }

    return 0;
}

void move_pioneer(int argc, char **argv, int len, int col[50], int row[50])
{
    int cnt = len - 1;
    std::cout << cnt << std::endl;

    Pioneer pioneer(argc, argv);

    Pioneer::direction direction = Pioneer::FORWARD;

    pioneer.stop();
    pioneer.upgrade();
    sleep(3);
    char key;
    while (ros::ok())
    {
        if (cnt < 0)
        {
            break;
        }

        std::cout << row[cnt] << " y " << col[cnt] << "cnt es:" << cnt << std::endl;
        std::cout << direction << std::endl;
        if (row[cnt] == 1 && col[cnt] == 0)
        {
            if (direction == Pioneer::FORWARD)
            {
                pioneer.go_forward();
            }
            else if (direction == Pioneer::LEFT)
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else if (direction == Pioneer::RIGHT)
            {
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else
            {
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_forward();
            }
            direction = Pioneer::FORWARD;
        }
        else if (row[cnt] == 1 && col[cnt] == 1)
        {
            if (direction == Pioneer::FORWARD)
            {
                //pioneer.turn_for45();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::LEFT)
            {
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::RIGHT)
            {
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                // pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                // pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_for45();
            }
            direction = Pioneer::FORWARD;
        }
        else if (row[cnt] == 0 && col[cnt] == 1)
        {

            if (direction == Pioneer::FORWARD)
            {
                pioneer.turn_for45();
                // pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else if (direction == Pioneer::LEFT)
            {
                pioneer.go_forward();
            }
            else if (direction == Pioneer::RIGHT)
            {
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_forward();
            }

            direction = Pioneer::LEFT;
        }

        else if (row[cnt] == 0 && col[cnt] == -1)
        {
            if (direction == Pioneer::FORWARD)
            {
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else if (direction == Pioneer::RIGHT)
            {
                pioneer.go_forward();
            }
            else if (direction == Pioneer::LEFT)
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else
            {
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_forward();
            }

            direction = Pioneer::RIGHT;
        }

        else if (row[cnt] == -1 && col[cnt] == 0)
        {
            if (direction == Pioneer::FORWARD)
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else if (direction == Pioneer::RIGHT)
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else if (direction == Pioneer::LEFT)
            {
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_forward();
            }
            else
            {
                pioneer.go_forward();
            }

            direction = Pioneer::BACKWARD;
        }
        else if (row[cnt] == -1 && col[cnt] == 1)
        {
            if (direction == Pioneer::FORWARD)
            {
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::RIGHT)
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::LEFT)
            {
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else
            {
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }

            direction = Pioneer::LEFT;
        }

        else if (row[cnt] == 1 && col[cnt] == -1)
        {
            if (direction == Pioneer::FORWARD)
            {
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::RIGHT)
            {

                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::LEFT)
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else
            {

                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }

            direction = Pioneer::RIGHT;
        }

        else if (row[cnt] == -1 && col[cnt] == -1)
        {
            if (direction == Pioneer::FORWARD)
            {
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop();
                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::RIGHT)
            {

                pioneer.turn_back45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }
            else if (direction == Pioneer::LEFT)
            {

                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                //pioneer.stop();
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
                //pioneer.stop()
            }
            else
            {
                pioneer.turn_for45();
                pioneer.stop();
                pioneer.go_diag();
                pioneer.stop();
                pioneer.turn_back45();
            }

            direction = Pioneer::BACKWARD;
        }

        else
        {
            pioneer.stop();
        }

        cnt--;
        //pioneer.stop();
    }
}