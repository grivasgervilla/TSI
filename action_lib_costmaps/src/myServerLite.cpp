// Inspirado en http://robot.wpi.edu/wiki/index.php/Actionlib



#include <ros/ros.h>
#include "myPlannerLite.h"
#include <nav_msgs/Odometry.h> //lo pongo porque vamos a manejar tb. datos de odometría.
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

//Clase que contiene el action server
class MyActionServer
{

protected:

    ros::NodeHandle n;
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as;

    move_base_msgs::MoveBaseFeedback 	feedback;
    move_base_msgs::MoveBaseResult 	result;
    move_base_msgs::MoveBaseGoal		goal;
    std::string				action_name;

    //odometría
    //nav_msgs::Odometry odometria; //se rellena con un callback de la subscripción.
    LocalPlanner planner; //el server tiene asociado un objeto planner, es el que
                          //desarrolla el comportamiento de navegar hasta el objetivo.


public:
    MyActionServer(std::string name):
        //Crea y configura el action server. Necesita una referencia al manejador del nodo ros, n
        //Se subscribe al topic "mi_move_base",
        // Y registra una callback mediante el puntero a la función ejecutaCB
        // la especificación de la callback es con la sintaxis boost:: porque estamos dentro
        // de una especificación de server como una clase C++
        as(n,"mi_move_base",
           boost::bind(&MyActionServer::ejecutaCB, this, _1), false),
        action_name(name)
    {
        //Registra Callbacks.
        as.registerPreemptCallback(boost::bind(&MyActionServer::preemptCB,this));
        //Inicia el server

        as.start();
        ROS_INFO("Action Server Lanzado");


    }

    //Callback para manejar preemption. ¿Reset your helpers here?
    void preemptCB()
    {
        ROS_INFO("%s está preempted", action_name.c_str());
        //no pongo result porque MoveBaseAction no lo tiene definido.

        //Entiendo que el server pondrá el status adecuado. Imagino que habrá que hacerlo
        as.setPreempted(result,"Goal en estado preempted");
    }

    //Callback para procesar el goal
    void ejecutaCB(const move_base_msgs::MoveBaseGoalConstPtr& goal)
    {
        //Si el servidor está muerto, no procesar
        if (!as.isActive() || as.isPreemptRequested())
        {
            ROS_INFO("El servidor está muerto.");
            return;
        }

        //Configurar la ejecución del procesado del goal a 5Hz
        ros::Rate rate(5);

        ROS_INFO("Procesando el goal. Enviando goal al navegador local");

        //Le pasamos al planner el objetivo recibido!!!
        planner.setGoal(goal);

        //procesamiento del goal
        bool success = true;

        while (true)
        {
            //Comprobar si ROS está vivo
            if (!ros::ok())
            {
                success = false;
                ROS_INFO("%s apagando ", action_name.c_str());
                break;
            }

            //Si el action server está muerto o preempted, detener procesamiento

            if(!as.isActive() || as.isPreemptRequested())
                return;

            //Publicar una actualización de status.
            //Informar del goal , obteniendo los datos del goal enviado
            //Estructura detallada de MoveBaseGoal, Feedback,... en la URL
            //http://ftp.isr.ist.utl.pt/pub/roswiki/doc/api/move_base_msgs/html/msg/MoveBaseGoal.html
            ROS_INFO("...yendo hacia el goal (%f,%f,%f)",
                     goal->target_pose.pose.position.x,
                     goal->target_pose.pose.position.y,
                     goal->target_pose.pose.orientation.w );

            ROS_INFO("Estoy en la posición:(%f, %f), yaw: %f", planner.pos.x, planner.pos.y,planner.yaw);
            planner.setDeltaAtractivo();
            //********************************************************
            //aquí habrá que mejorar el navegador local para que evite obstáculos
            //con el componente repulsivo
            //****************************************************************
            planner.setDeltaTotal();
            planner.setv_Angular();
            planner.setv_Lineal();
            if (planner.goalAchieved())
                break;
            planner.setSpeed();
            ROS_INFO("Enviando velocidad (%f,%f)", planner.v_lineal, planner.v_angular);

            //Informar del feedback. Consiste en obtener información
            //de la odometría y pasarla al formato de MoveBaseFeedback
            // En http://ros-users.122217.n3.nabble.com/Subscribing-to-PR2-Base-Odometry-td689732.html
            // hay un ejemplo de como subscribirse a la odometría.

            //*** Rellenar feedback con odometría ******//!!!!!!

            feedback.base_position.pose.position.x = planner.odometria.pose.pose.position.x;
            feedback.base_position.pose.position.y = planner.odometria.pose.pose.position.y;
            feedback.base_position.pose.orientation.w = planner.odometria.pose.pose.orientation.w;

            ROS_INFO("...con feedback (%f,%f,%f)",
                     feedback.base_position.pose.position.x,
                     feedback.base_position.pose.position.y,
                     feedback.base_position.pose.orientation.w);

            as.publishFeedback(feedback);
            rate.sleep();
        }
        //Publicar el resultado si el goal no está preempted
        if (success)
        {
            ROS_INFO("%s tuvo éxito al llegar al goal (%f %f %f)", action_name.c_str(),
                     goal->target_pose.pose.position.x,
                     goal->target_pose.pose.position.y,
                     goal->target_pose.pose.orientation.w );
            as.setSucceeded();
        }
        else
            as.setAborted();






    }
};

//Used by ROS to actually create the node. Could theoretically spawn more than one server

int main(int argc, char** argv)

{

    ros::init(argc, argv, "mi_move_base");



    //Just a check to make sure the usage was correct

    if(argc != 1)
    {

        ROS_INFO("Usage: test_server");

        return 1;

    }

    //Spawn the server

    MyActionServer server(ros::this_node::getName());



    ros::spin();

    return 0;

}
