#ifndef LOCAL PLANNER_H
#define LOCAL PLANNER_H


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <iostream>

using namespace std;

typedef struct tupla {double x; double y;} Tupla;
typedef struct conf {double radius; double spread; double intens;
                     } PFConf;


template <typename T> int signo(T val) {
    return  (val/abs(val));
    }

class LocalPlanner
{
    public:
        Tupla posGoal;  //Posición del objetivo
        Tupla pos;      //Posición actual
        double yaw;     //Angulo (en radianes) de orientación del robot
        Tupla deltaGoal;//Componente del campo atractivo
        Tupla deltaObst;//Componente del campo repulsivo (para todos los obstáculos)
        Tupla delta;    //Componente total
        double v_angular; //velocidad angular
        double v_lineal;  //velocidad lineal
        double tolerancia_llegada = 0.0;
	int veces_block = 0;  //veces que se ha quedado bloqueado
	bool espera_plan = false;
        std::vector<Tupla> posObs;    //Vector que contiene las posiciones de los obstáculos.
                                        //Calculado en scanCallback.
        nav_msgs::Odometry odometria;      //guarda el último mensaje de odometría recibido
		  bool status_suicida = false; //lo ponemos a true si el robot entra en la zona donde tiene a suicidarse
		  bool status_block = false;
		  Tupla dir_salvamento;

        PFConf CAMPOATT;// = {0.01,3,5,0.07};//Parámetros de configuración (radio, spread, alpha) del campo actractivo.
        PFConf CAMPOREP;//(0,01,1,0,01);//Parámetros de configuración (radio, spread, beta)del campo repulsivo.
        //CAMPOREP.radius = 0.0001; //nuevo radio del campo repulsivo
        const static double TOLERANCIA = 0.09;  //Valor a partir del cual consideramos que el robot está e
                              //en la posición objetivo (ver setDeltaAtractivo)
        const static double V_ANGULAR_CTE = M_PI/8;  //Valor de la velocidad angular constante.
        const static double EPSILON_ANGULAR = 0.0009; //Valor a partir del cual entendemos que el robot está en la orientación deseada
        const static double MIN_SCAN_ANGLE_RAD = -90.0/180*M_PI;
        const static double MAX_SCAN_ANGLE_RAD = +90.0/180*M_PI;
        LocalPlanner(); //constructor.
        void setGoal(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
            posGoal.x = goal->target_pose.pose.position.x;
            posGoal.y = goal->target_pose.pose.position.y;
            }

        bool goalAchieved();    //Devuelve true cuando se ha alcanzado el objetivo
        void setDeltaAtractivo();
        void getOneDeltaRepulsivo(Tupla posO, Tupla &deltaO);
        void setTotalRepulsivo();
        void setDeltaTotal(){
		double modulo_goal, modulo_obst;

		if (norm(deltaObst) < 0.00001 && status_block){
			status_block = false;
			veces_block++;
		}

		delta.x = deltaGoal.x + deltaObst.x;
		delta.y = deltaGoal.y + deltaObst.y;

		modulo_goal = norm(deltaGoal);
		modulo_obst = norm(deltaObst);

		if (modulo_obst > 10*modulo_goal and !status_suicida) {
			status_suicida = true;
			dir_salvamento.x = delta.x;
			dir_salvamento.y = delta.y;
		}

		if (norm(delta) < 0.00001 and norm(deltaObst) >= 0.00001){
			status_block = true;

		}

		if (status_block) {
			delta.x = deltaObst.x;
			delta.y = deltaObst.y;
		}
		if (veces_block > 2){
			veces_block = 0;
			espera_plan = true;
		}
	};
        void setv_Angular();
        void setv_Lineal();

        double distancia(Tupla src, Tupla dst) {
            return sqrt((src.x - dst.x) * (src.x - dst.x) +
                        (src.y - dst.y) * (src.y - dst.y));
            }
        void setSpeed() {
        //rellenar y enviar Twist.
            geometry_msgs::Twist mensajeTwist;
            mensajeTwist.linear.x = v_lineal;
            mensajeTwist.angular.z =v_angular;
            commandPub.publish(mensajeTwist);
            }

    protected:
    private:
        ros::NodeHandle node;
        ros::Publisher commandPub;  //Publicador de velocidades
        ros::Subscriber laserSub;   //Suscriptor del scan laser
        ros::Subscriber odomSub;    //Suscriptor de la odometría.

        void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
        void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
        double norm (Tupla p) {
        		return sqrt(p.x*p.x+p.y*p.y);
        }

};

#endif // LOCAL PLANNER_H
