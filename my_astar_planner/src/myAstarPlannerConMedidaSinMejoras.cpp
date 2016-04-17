/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2015, Juan Fdez-Olivares
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Juan Fdez-Olivares, Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

// para debugging
#include <sstream>
#include <string>







//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

  //devuelve un puntero a un nodo en una lista de nodos (nodo = coupleOfCells) a partir del índice del nodo
  list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID);

  //comprueba si un índice de nodo existe en una lista de nodos.
  bool isContains(list<coupleOfCells> & list1, int cellID);

  MyastarPlanner::MyastarPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  //inicializador del global_planner, mejor no tocar nada.
  void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      //vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      //world_model_ = new base_local_planner::CostmapModel(*costmap_);

      //el plan se va a publicar en el topic "planTotal"
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //esta función puede usarse para ayudar a calcular rutas seguras
  //está preparada para obtener el footprint del robot y devolver un valor representando el coste de la posición del robot.
  double MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    double x,y;
    unsigned int mx, my;
    int n_no_free = 0;
    //if we have no footprint... do nothingfootprintCost
    if(footprint.size() < 3)
      return -1.0;

    //Recorremos las celdas en las que toca el borde del footprint del robot centrado en el punto que le pasamos
    for (int i = 0; i < footprint.size(); i++) {
      x = x_i + (footprint.at(i).x*cos(theta_i) - footprint.at(i).y*sin(theta_i));
      y = y_i + (footprint.at(i).x*sin(theta_i) + footprint.at(i).y*cos(theta_i));
      costmap_->worldToMap(x,y,mx, my);

      //Vemos si es un obstaculo o una posicion peligrosa
      if(costmap_->getCost(mx,my) == costmap_2d::LETHAL_OBSTACLE)
        return 999999999;
      else if (costmap_->getCost(mx,my) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        n_no_free++;
    }

    double cte = 1.0; //TODO
    return cte*n_no_free;
  }

  // Funciones hechas por los alumnos para completar el codigo
  /*
  Funcion que devuelve un iterador al elemento buscado en la lista.
  Si no se encuentra, devuelve end.
  */
  list<coupleOfCells>::iterator findNodo (unsigned int index, list<coupleOfCells> &nodos) {
  	for(list<coupleOfCells>::iterator it = nodos.begin(); it != nodos.end(); it++) {
      if (it->index == index)
        return it;
    }
    return nodos.end();
  }

  /*
  Funcion que saca el mejor nodo de la lista de abiertos y lo devuelve
  */
  coupleOfCells getMejorNodo (list<coupleOfCells> &OPL) {
  	double f_min = 999999999;
    list<coupleOfCells>::iterator it_min;

    for( list<coupleOfCells>::iterator it = OPL.begin(); it != OPL.end(); it++) {
      if (it->fCost < f_min) {
        f_min = it->fCost;
        it_min = it;
      }
    }

  	coupleOfCells mejor_nodo = *it_min;
  	OPL.erase(it_min);

  	return mejor_nodo;

  }

  //función llamada por move_base para obtener el plan.
  bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    //***********************************************************
    // Inicio de gestion de ROS, mejor no tocar nada en esta parte
    //***********************************************************
    if(!initialized_){
      ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_INFO("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    MyastarPlanner::openList.clear();
    MyastarPlanner::closedList.clear();
    plan.clear();

    //obtenemos el costmap global  que está publicado por move_base.
    costmap_ = costmap_ros_->getCostmap();


    //Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
    //esto es importante para evitar errores de transformaciones de coordenadas.
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    //obtenemos la orientación start y goal en start_yaw y goal_yaw.
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);


    /**************************************************************************/
    /*************** HASTA AQUÍ GESTIÓN DE ROS *********************************/
    /****************************************************************************/

    //pasamos el goal y start a un nodo (estructura coupleOfCells)
    coupleOfCells cpstart, cpgoal;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
    cpgoal.parent=0;
    cpgoal.gCost=0;
    cpgoal.hCost=0;
    cpgoal.fCost=0;

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;
    costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
    cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
    cpstart.parent =cpstart.index;
    cpstart.gCost = 0;
    cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);

    //insertamos el nodo inicial en abiertos
    MyastarPlanner::openList.push_back(cpstart);


    ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
    ROS_INFO("Index del goal: %d", cpgoal.index );

    unsigned int explorados = 0;
    unsigned int currentIndex = cpstart.index;

    while (!MyastarPlanner::openList.empty()) //while the open list is not empty continuie the search
    {

        //escoger el nodo (coupleOfCells) de abiertos que tiene el valor más pequeño de f.
        coupleOfCells COfCells= getMejorNodo(MyastarPlanner::openList);
        currentIndex=COfCells.index;

        //vamos a insertar ese nodo  en cerrados

            //obtenemos un iterador a ese nodo en la lista de abiertos
            //list<coupleOfCells>::iterator it=getPositionInList(openList,currentIndex);


            //copiamos el contenido de ese nodo a una variable nodo auxiliar
            cpstart.index=currentIndex;
            cpstart.parent=COfCells.parent;
            cpstart.gCost=COfCells.gCost;
            cpstart.hCost=COfCells.hCost;
            cpstart.fCost=COfCells.fCost;


        //y esa variable la insertamos en cerrados
            MyastarPlanner::closedList.push_back(cpstart);
        //ROS_INFO("Inserto en CERRADOS: %d", (*it).index );
           ROS_INFO("Mi G: %f, H: %f, F: %f", COfCells.gCost, COfCells.hCost, COfCells.fCost);
           ROS_INFO("Index: %d Parent: %d", COfCells.index, COfCells.parent);



          // Si el nodo recién insertado es el goal, ¡plan encontrado!

          if(currentIndex==cpgoal.index  || explorados == 2000)
          {
            //el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
            //y vamos insertando al final los waypoints (los nodos de cerrados)

              ROS_INFO("PLAN ENCONTRADO!!!");

            //convertimos goal a poseStamped nueva


              geometry_msgs::PoseStamped pose;
              pose.header.stamp =  ros::Time::now();
              pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el goal pasado por parámetro
              pose.pose.position.x = goal_x;
              pose.pose.position.y = goal_y;
              pose.pose.position.z = 0.0;
              pose.pose.orientation.x = 0.0;
              pose.pose.orientation.y = 0.0;
              pose.pose.orientation.z = 0.0;
              pose.pose.orientation.w = 1.0;

              //lo añadimos al plan%
              plan.push_back(pose);
              ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);

              coupleOfCells currentCouple = cpstart;
              unsigned int currentParent = cpstart.parent;

              while (currentCouple.index != currentParent) //e.d. mientras no lleguemos al nodo start
              {
                //encontramos la posición de currentParent en cerrados

                 list<coupleOfCells>::iterator it=getPositionInList(MyastarPlanner::closedList,currentParent);
                //hacemos esa posición que sea el currentCouple
                currentCouple.index=currentParent;
                currentCouple.parent=(*it).parent;
                currentCouple.gCost=(*it).gCost;
                currentCouple.hCost=(*it).hCost;
                currentCouple.fCost=(*it).fCost;

                //creamos una PoseStamped con la información de currentCouple.index

                        //primero hay que convertir el currentCouple.index a world coordinates
                unsigned int mpose_x, mpose_y;
                double wpose_x, wpose_y;

                costmap_->indexToCells(currentCouple.index, mpose_x, mpose_y);
                costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);


                        //después creamos la pose
                geometry_msgs::PoseStamped pose;
                pose.header.stamp =  ros::Time::now();
                pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
                pose.pose.position.x = wpose_x;
                pose.pose.position.y = wpose_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                //insertamos la pose en el plan
                plan.push_back(pose);
                ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
                //hacemos que currentParent sea el parent de currentCouple
                currentParent = currentCouple.parent;
              }

            ROS_INFO("Sale del bucle de generación del plan.");
            ROS_WARN("Numero de nodos explorados: %d", explorados);
            std::reverse(plan.begin(),plan.end());

            //lo publica en el topic "planTotal"
            publishPlan(plan);
            return true;
          }

          //Si no hemos encontrado plan aún eliminamos el nodo insertado de ABIERTOS.
          //openList.pop_front();
          unsigned int current_mx, current_my;
          double current_wx, current_wy;
          //Buscamos en el costmap las celdas adyacentes a la actual
          costmap_->indexToCells(currentIndex, current_mx, current_my);
          costmap_->mapToWorld(current_mx, current_my, current_wx, current_wy);

          double distance = sqrt((goal_x - current_wx)*(goal_x - current_wx) + (goal_y - current_wy)*(goal_y - current_wy));
          vector <unsigned int> neighborCells=findFreeNeighborCell(currentIndex, distance);
          vector <unsigned int> new_open_nodes_idx;
          //Ignoramos las celdas que ya existen en CERRADOS
          for (int i = 0; i < neighborCells.size(); i++) {
      			list<coupleOfCells>::iterator it_nodo_en_cerrados = findNodo(neighborCells.at(i), MyastarPlanner::closedList);
      			list<coupleOfCells>::iterator it_nodo_en_abiertos = findNodo(neighborCells.at(i), MyastarPlanner::openList);

      			if (it_nodo_en_cerrados == MyastarPlanner::closedList.end() && it_nodo_en_abiertos == MyastarPlanner::openList.end()) { //si el nodo no esta en abiertos ni en cerrados
      				new_open_nodes_idx.push_back(neighborCells.at(i));
      			}
      			// else if (idx_nodo_en_cerrados != -1) { //si esta en cerrados
      			// 	if (closedList.at(idx_nodo_en_cerrados).gCost > neighborCells.at(i).gCost){ //si la g que tenia es peor que la nueva actualizo datos
      			// 		closedList.at(idx_nodo_en_cerrados) = neighborCells.at(i);
      			// 	}
      			// }
      			else if(it_nodo_en_abiertos != MyastarPlanner::openList.end()){ //esta en abiertos
                 double new_g_cost = cpstart.gCost + getMoveCost(currentIndex, neighborCells.at(i));
      			     if (it_nodo_en_abiertos->gCost > new_g_cost) { //si la g que tenia es peor que la nueva actualizo datos
                       it_nodo_en_abiertos->gCost = new_g_cost;
                       it_nodo_en_abiertos->fCost = new_g_cost + it_nodo_en_abiertos->hCost;
                       it_nodo_en_abiertos->parent = currentIndex;
      			     }
      			}
      		}


          //Determinamos las celdas que ya están en ABIERTOS y las que no están en ABIERTOS



         //Añadimos a ABIERTOS las celdas que todavía no están en ABIERTO, marcando el nodo actual como su padre
         //ver la función addNeighborCellsToOpenList(openList, neighborsNotInOpenList, currentIndex, coste_del_nodo_actual, indice_del_nodo_goal);
         addNeighborCellsToOpenList(MyastarPlanner::openList, new_open_nodes_idx, currentIndex, cpstart.gCost, cpgoal.index);
         explorados++;
    }

    if(MyastarPlanner::openList.empty())  // if the openList is empty: then failure to find a path
        {
            ROS_INFO("Failure to find a path !");
            return false;
           // exit(1);
        }

};


//calculamos H como la distancia euclídea hasta el goal
double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
  unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
  double wstart_x, wstart_y, wgoal_x, wgoal_y;

  //trasformamos el indice de celdas a coordenadas del mundo.
  //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

  costmap_->indexToCells(start, mstart_x, mstart_y);
  costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
  costmap_->indexToCells(goal, mgoal_x, mgoal_y);
  costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

  double distance = sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2));

  return distance + footprintCost(wstart_x, wstart_y, 0.0);
 }


//comparamos F para dos nodos.
bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2)
{
   return c1.fCost < c2.fCost;
}

/*******************************************************************************/
//Function Name: getPositnionInList
//Inputs:the cellID, the list
//Output: index of the cell in the list
//Description: it is used to search the index of a cell in a list
/*********************************************************************************/
list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID)
{
   for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
     if (it->index == cellID)
         return it;
   }
}


 /*******************************************************************************
 * Function Name: findFreeNeighborCell
  * Inputs: el índice de la celda
  * Output: a vector of free neighbor cells of the current cell
  * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
  * findFreeNeighborCell(i) es el conjunto de celdas del costmap tales que
  *             costmap[x,y] = FREE_SPACE, donde (x,y) son las coordenadas en el costmap del índice i
  *
*********************************************************************************/
vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID, double distance){

        unsigned int mx, my;
        costmap_->indexToCells(CellID,mx,my);
        vector <unsigned int>  freeNeighborCells;
        int paso = 1;

        for (int x=-1;x<=1;x++)
          for (int y=-1; y<=1;y++){
            //check whether the index is valid
           if ((mx+paso*x>=0)&&(mx+paso*x < costmap_->getSizeInCellsX())&&(my+paso*y >=0 )&&(my+paso*y < costmap_->getSizeInCellsY())){
              if(costmap_->getCost(mx+paso*x,my+paso*y) == costmap_2d::FREE_SPACE   && (!(x==0 && y==0))){
                  unsigned int index = costmap_->getIndex(mx+paso*x,my+paso*y);
                  freeNeighborCells.push_back(index);
              }
            }
        }
          return  freeNeighborCells;

      }


/*******************************************************************************/
//Function Name: isContains
//Inputs: the list, the cellID
//Output: true or false
//Description: it is used to check if a cell exists in the open list or in the closed list
/*********************************************************************************/
 bool isContains(list<coupleOfCells> & list1, int cellID)
 {
   for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
     if (it->index == cellID)
         return true;
  }
   return false;
}

double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
  //calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
  return calculateHCost(here,there);

}

/*******************************************************************************/
//Function Name: addNeighborCellsToOpenList
//Inputs: the open list, the neighbors Cells and the parent Cell
//Output:
//Description: it is used to add the neighbor Cells to the open list
/*********************************************************************************/
void MyastarPlanner::addNeighborCellsToOpenList(list<coupleOfCells> & OPL, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell) //,float tBreak)
{
        vector <coupleOfCells> neighborsCellsOrdered;
        for(uint i=0; i< neighborCells.size(); i++)
        {
          coupleOfCells CP;
          CP.index=neighborCells[i]; //insert the neighbor cell
          CP.parent=parent; //insert the parent cell
          //calculate the gCost
          CP.gCost=gCostParent+getMoveCost(parent,neighborCells[i]);

          //calculate the hCost: Euclidian distance from the neighbor cell to the goalCell
          CP.hCost=calculateHCost(neighborCells[i],goalCell);
          //calculate fcost

          CP.fCost=CP.gCost+CP.hCost;
         // neighborsCellsOrdered.push_back(CP);
          OPL.push_back(CP);
        }
      }

//publicamos el plan para poder visualizarlo en rviz
void MyastarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
          if (!initialized_) {
              ROS_ERROR(
                      "This planner has not been initialized yet, but it is being used, please call initialize() before use");
              return;
          }

          //create a message for the plan
          nav_msgs::Path gui_path;
          gui_path.poses.resize(path.size());

          if (!path.empty()) {
              gui_path.header.frame_id = path[0].header.frame_id;
              gui_path.header.stamp = path[0].header.stamp;
          }

          // Extract the plan in world co-ordinates, we assume the path is all in the same frame
          for (unsigned int i = 0; i < path.size(); i++) {
              gui_path.poses[i] = path[i];
          }

          plan_pub_.publish(gui_path);
      }

}
