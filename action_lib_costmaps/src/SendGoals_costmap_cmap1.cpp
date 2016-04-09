#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//gestion de costmaps
#include "tf/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include <costmap_2d/costmap_2d.h>
#include <vector>


#include <boost/thread.hpp>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double prev_pos_x,prev_pos_y, goal_x, goal_y, prev_yaw;
bool condicion_parada;
int tiempo_parado = 0;
const int TOPE_TIEMPO_PARADO = 25;

//imprime en pantalla el contendio de un costmap
void printCostmap (costmap_2d::Costmap2DROS * costmap_ros) {
     //typedef basic_string<unsigned char> ustring;


     costmap_2d::Costmap2D *costmap;
     costmap = costmap_ros-> getCostmap();

     for (int x=0 ;x < costmap->getSizeInCellsX() ;x++)
          for (int y= 0; y < costmap->getSizeInCellsY();y++){
              std::cout << (int) costmap->getCost(x,y) << " ";
            }
        }


//
//  /*******************************************************************************
// * Function Name: findFreeNeighborCell
//  * Inputs: the row and columun of the current Cell
//  * Output: a vector of free neighbor cells of the current cell
//  * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
//  * Check Status: Checked by Anis, Imen and Sahar
//*********************************************************************************/
std::vector <unsigned int> findFreeNeighborCell (unsigned int CellID, costmap_2d::Costmap2D *costmap){
//
        unsigned int mx, my;
        //convertir el índice de celda en coordenadas del costmap
        costmap->indexToCells(CellID,mx,my);
        ROS_INFO("CellID = %d, mx = %d, my= %d", CellID, mx, my);
        std::vector <unsigned int>  freeNeighborCells;

        for (int x=-1;x<=1;x++)
          for (int y=-1; y<=1;y++){
            //comprobar si el índice es válido
             if ((mx+x>=0)&&(mx+x < costmap->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap->getSizeInCellsY())){

              ROS_WARN("Index valid!!!!! costmap[%d,%d] = %d", mx+x,my+y,costmap->getCost(mx+x,my+y));
              if(costmap->getCost(mx+x,my+y) == costmap_2d::FREE_SPACE   && (!(x==0 && y==0))){
                  unsigned int index = costmap->getIndex(mx+x,my+y);
                  ROS_INFO("FREECELL: %d con valor %d", index, costmap_2d::FREE_SPACE);
                  freeNeighborCells.push_back(index);
              }
              else {
                  unsigned int index = costmap->getIndex(mx+x,my+y);
                  ROS_INFO("OCCUPIEDCELL: %d con valor %d", index, costmap->getCost(mx+x,my+y));
                 }
           }
        }
          return  freeNeighborCells;

      }
//función para rellenar una PoseStamped a partir de las coordenadas del mundo obtenidas del costmap.

void rellenaPoseStamped (double wx, double wy, geometry_msgs::PoseStamped &pose, costmap_2d::Costmap2DROS *costmap){

    pose.header.stamp =  ros::Time::now();
    //el marco de coordendas de la posición debe ser el mismo frame que el del goal y este, a su vez, el mismo que el del costmap, por defecto debe tenerlo
    pose.header.frame_id = costmap->getGlobalFrameID();
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0; //la orientación en principio no interesa.
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    }

struct Nodo {
	unsigned int id;
	unsigned int padre;
	double g;
	double f;
};

Nodo crearNodo(unsigned int id, unsigned int padre, double g, double f) {
	Nodo nuevo;
	nuevo.id = id;
	nuevo.padre = padre;
	nuevo.g = g;
	nuevo.f = f;

	return nuevo;
}


//Funcion que calcula la distancia estimada de un nodo a la meta
double h(unsigned int id, unsigned int meta, costmap_2d::Costmap2D *costmap) {
	unsigned int id_x, id_y, meta_x, meta_y;
	costmap->indexToCells(id, id_x, id_y);
	costmap->indexToCells(meta, meta_x, meta_y);

	double dist = sqrt((id_x-meta_x)*(id_x-meta_x)+(id_y-meta_y)*(id_y-meta_y));

	return dist;

}

Nodo getMejorNodo (vector<Nodo> &nodos) {
	double f_min = 9999999999999999999;
	int id_min;

	for (int i = 0; i < nodos.size(); i++)
		if (nodos.at(i).f < f_min) {
			f_min = nodos.at(i).f;
			id_min = i;
		}

	Nodo mejor_nodo = nodos.at(id_min);
	nodos.erase(nodos.begin() + id_min);

	return mejor_nodo;

}

vector<Nodo> getHijos (Nodo nodo, unsigned int meta, costmap_2d::Costmap2D *costmap) {
	vector<unsigned int> hijos_id = findFreeNeighborCell (nodo.id, costmap);
	vector<Nodo> nodos_hijos;
	double g,f;
	ROS_INFO("TAMAÑO DEL VECTOR DE HIJOS: %d", hijos_id.size());
	for (int i = 0; i < hijos_id.size(); i++) {
		g = nodo.g + 1.0;
		f = h(hijos_id.at(i), meta, costmap) + g;
		nodos_hijos.push_back(crearNodo(hijos_id.at(i), nodo.id, g, f));
	}

	return nodos_hijos;
}

int findNodo (unsigned int id, vector<Nodo> &nodos) {
	for (int i = 0; i < nodos.size(); i++)
		if (nodos.at(i).id == id)
			return i;

	return -1;
}

vector<geometry_msgs::PoseStamped> fabricarCamino(vector<Nodo> &cerrados, unsigned int start, costmap_2d::Costmap2DROS *costmap_ros) {
	vector<int> idx_camino;
	int idx_padre;
	bool fin = false;
	double wx, wy;
	unsigned int mx, my;
	costmap_2d::Costmap2D *costmap = costmap_ros->getCostmap();

	//Creamos un camino dado por indices desde el final al principio
	unsigned int padre = cerrados.at(cerrados.size()-1).padre;
	idx_camino.push_back(cerrados.back().id);

	while (!fin) {
		idx_padre = findNodo(padre, cerrados);

		idx_camino.push_back(cerrados.at(idx_padre).id);
		padre = cerrados.at(idx_padre).padre;

		if (padre == start)
			fin = true;
	}

	//idx_camino.push_back(start);

	//Creamos el camino de PoseStamped de principio a fin
	vector<geometry_msgs::PoseStamped> camino (idx_camino.size());

	for (int i = 0; i < idx_camino.size(); i++) {
		costmap->indexToCells(idx_camino.at(i), mx, my);
    ROS_INFO("Mete en el camino los indices: %d,%d",mx,my);
		costmap->mapToWorld(mx, my, wx, wy);
    ROS_INFO("En el mundo son: %f,%f",wx,wy);
		rellenaPoseStamped(wx, wy, camino.at(i), costmap_ros);

	}

	return camino;

}


std::vector<geometry_msgs::PoseStamped> A_estrella(unsigned int start, unsigned int goal, costmap_2d::Costmap2DROS *costmap_ros) {
	ROS_INFO("ENTRO EN EL A ESTRELLA");
	vector<geometry_msgs::PoseStamped> camino;
	vector<Nodo> abiertos, cerrados, sucesores;
	bool fin = false;
	Nodo nodo_expand;
	costmap_2d::Costmap2D *costmap = costmap_ros->getCostmap();

	//Quiero imprimir el mapa
	//costmap_2d::Costmap2D *costmap;
     	//costmap = costmap_ros-> getCostmap();

	ROS_INFO("Aqui si que es dificil encontrar el mapa DORA!");
     	for (int x=0 ;x < costmap->getSizeInCellsX() ;x++)
          for (int y= 0; y < costmap->getSizeInCellsY();y++){
              ROS_INFO("%d",costmap->getCost(x,y));
          }
	unsigned int mx, my;
	costmap->indexToCells(goal, mx, my);
	ROS_INFO("mi goal es %d, %d", mx, my);

	Nodo inicial = crearNodo(start, 0, 0.0, h(start, goal, costmap));
	abiertos.push_back(inicial);

	while (!fin && abiertos.size() > 0) {
		ROS_INFO("ENTRO EN EL WHILE");
		nodo_expand = getMejorNodo(abiertos);
		ROS_INFO("ID DEL NODO A EXPANDIR: %d", nodo_expand.id);

		//comprobamos si el nodo extraido es el final
		if (nodo_expand.id == goal)
			fin = true;

		//generamos los hijos de este nodo
		sucesores = getHijos(nodo_expand, goal, costmap);
		ROS_INFO("TAMAÑO DE SUCESORES: %d", sucesores.size());
		for (int i = 0; i < sucesores.size(); i++) {
			ROS_INFO("ENTRO EN EL FOR");
			int idx_nodo_en_cerrados = findNodo(sucesores.at(i).id, cerrados);
			ROS_INFO("Ya he mirado en cerrados idx vale: %d", idx_nodo_en_cerrados);
			int idx_nodo_en_abiertos = findNodo(sucesores.at(i).id, abiertos);
			ROS_INFO("Ya he mirado en abiertos idx vale: %d", idx_nodo_en_abiertos);

			if (idx_nodo_en_cerrados == -1 && idx_nodo_en_abiertos == -1) {//si el nodo no esta en abiertos ni en cerrados
				ROS_INFO("Ni en abiertos ni en cerrados.");
				abiertos.push_back(sucesores.at(i));
			}
			else if (idx_nodo_en_cerrados != -1) {//si esta en cerrados
				ROS_INFO("Esta en cerrados.");
				if (cerrados.at(idx_nodo_en_cerrados).g > sucesores.at(i).g){ //si la g que tenia es peor que la nueva actualizo datos
					ROS_INFO("Hay que cambiar la g de un cerrado.");
					cerrados.at(idx_nodo_en_cerrados) = sucesores.at(i);
				}
			}
			else {//esta en abiertos
				ROS_INFO("Esta en abiertos.");
				if (abiertos.at(idx_nodo_en_abiertos).g > sucesores.at(i).g) {//si la g que tenia es peor que la nueva actualizo datos
					ROS_INFO("Cambio el padre del nodo que estaba en abiertos.");

					abiertos.at(idx_nodo_en_abiertos) = sucesores.at(i);
				}
			}
		}
		ROS_INFO("SALGO DEL FOR");

		//introduzco el nodo que acabo de expandir en cerrados
		cerrados.push_back(nodo_expand);
		ROS_INFO("SALGO DEL WHILE");

	}

	ROS_INFO("TAMAÑO DE CERRADOS: %d", cerrados.size());

	//si hemos encontrado solucion la formateamos y devolvemos
	if (fin)
		camino = fabricarCamino(cerrados, start, costmap_ros);

	ROS_INFO("La longitud del camino devuelto es: %d", camino.size());
	return camino;
}



bool localPlanner(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2DROS *costmap_ros){


    ROS_INFO("localPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_2d::Costmap2D *costmap = costmap_ros->getCostmap();
    double distancia = 0.0;

    //pasamos el goal y start a coordenadas del costmap
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    unsigned int indice_goal;
    double p_x,p_y;
    //transformamos las coordenadas del mundo a coordenadas del costmap
    costmap->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    if ((mgoal_x>=0)&&(mgoal_x < costmap->getSizeInCellsX())&&(mgoal_y>=0 )&&(mgoal_y < costmap->getSizeInCellsY()))
        indice_goal = costmap->getIndex(mgoal_x, mgoal_y);
    else {
      double min_distancia = 5000000000;

      for(int i = 0; i < costmap->getSizeInCellsX(); i++){
        if (costmap->getCost(i,0) == costmap_2d::FREE_SPACE){
        	 costmap->mapToWorld(i,0,p_x,p_y);
        	 distancia = (p_x - goal_x)*(p_x - goal_x) + (p_y -goal_y)*(p_y -goal_y);
          if (distancia < min_distancia){
            min_distancia = distancia;
            indice_goal = costmap->getIndex(i,0);
          }
        }

        if (costmap->getCost(i,costmap->getSizeInCellsY()-1) == costmap_2d::FREE_SPACE){
          	costmap->mapToWorld(i,costmap->getSizeInCellsY()-1,p_x,p_y);
        	 	distancia = (p_x - goal_x)*(p_x - goal_x) + (p_y -goal_y)*(p_y -goal_y);
            if (distancia < min_distancia){
              min_distancia = distancia;
              indice_goal = costmap->getIndex(i,costmap->getSizeInCellsY()-1);
            }
        }
      }

      for(int i = 0; i < costmap->getSizeInCellsY(); i++){

        if (costmap->getCost(0,i) == costmap_2d::FREE_SPACE){
        	 costmap->mapToWorld(0,i,p_x,p_y);
        	 distancia = (p_x - goal_x)*(p_x - goal_x) + (p_y -goal_y)*(p_y -goal_y);
          if (distancia < min_distancia){
            min_distancia = distancia;
            indice_goal = costmap->getIndex(0,i);
          }
        }

        if (costmap->getCost(costmap->getSizeInCellsX()-1,i) == costmap_2d::FREE_SPACE){
        	 costmap->mapToWorld(costmap->getSizeInCellsX()-1,i,p_x,p_y);
        	 distancia = (p_x - goal_x)*(p_x - goal_x) + (p_y -goal_y)*(p_y -goal_y);
          if (distancia < min_distancia){
            min_distancia = distancia;
            indice_goal = costmap->getIndex(costmap->getSizeInCellsY()-1,i);
          }
        }
      }
    }

    //transformamos las coordenadas de la posición inicial a coordenadas del costmap.
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    ROS_INFO("Info del nodo inicial: valores del mundo ( %f,%f)",start_x,start_y);
    unsigned int mstart_x, mstart_y;
    costmap->worldToMap(start_x,start_y, mstart_x, mstart_y);
    ROS_INFO("Ros nos dice que eso, en el mapa es: (%d,%d)",mstart_x,mstart_y);
    unsigned int start_index = costmap->getIndex(mstart_x, mstart_y);
    ROS_INFO("Entonces, tenemos el ID: %d",start_index);
    unsigned int goal_index = costmap->getIndex(mgoal_x, mgoal_y);


    //*************************************************************************
    // ya tenemos transformadas las coordenadas del mundo a las del costmap,
    // ahora hay que implementar la búsqueda de la trayectoria, a partir de aquí.
    //*************************************************************************
    ROS_INFO("VOY A LLAMAR AL A ESTRELLA");
    plan = A_estrella(start_index, indice_goal, costmap_ros);
    }



/*bool localPlanner(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2DROS *costmap_ros){


    geometry_msgs::PoseStamped pose;
    rellenaPoseStamped(9,0,pose,costmap_ros);//Es el Costmap2DROS
    plan.push_back(pose);
  }*/




//consultado en http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient%28Threaded%29


void feedbackCBGoal0(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){

  double pos_x = feedback->base_position.pose.position.x;
  double pos_y = feedback->base_position.pose.position.y;
  double yaw = feedback->base_position.pose.orientation.w;

  if(pos_x == prev_pos_x && pos_y == prev_pos_y && yaw == prev_yaw)
    tiempo_parado++;
  else tiempo_parado = 0;

  if (tiempo_parado >= TOPE_TIEMPO_PARADO)
    condicion_parada = true;

  prev_pos_x = pos_x;
  prev_pos_y = pos_y;
  prev_yaw = yaw;

 // ROS_INFO("Estoy en la posición:(%f, %f), distancia: %f", pos_x, pos_y,
       //   sqrt((pos_x - goal_x) * (pos_x - goal_x) + (pos_y - goal_y) * (pos_y - goal_y)));
}

void doneCBGoal0(const actionlib::SimpleClientGoalState& estado, const move_base_msgs::MoveBaseResultConstPtr &resultado){
  if (estado == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Se ha conseguido el objetivo.");
  else if (estado == actionlib::SimpleClientGoalState::PREEMPTED)
    ROS_INFO("Se ha cancelado el objetivo.");
  else if (estado == actionlib::SimpleClientGoalState::ABORTED)
    ROS_INFO("Se ha abortado el objetivo.");
}

void activeCBGoal0(){
  ROS_INFO("El goal actual está activo");
}



void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv) {

  ros::init(argc,argv, "send_goals_node");
  //el cliente usará un costmap local para poder tener una representación del entorno
  //en su vecindad próxima.
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS* localcostmap = new costmap_2d::Costmap2DROS("local_costmap", tf);
  costmap_2d::Costmap2DROS* globalcostmap = new costmap_2d::Costmap2DROS("global_costmap", tf);

  //detener el funcionamiento de los costmaps hasta que estemos conectados con el servidor.

  localcostmap->pause();
  globalcostmap->pause();

  //costmap_2d::Costmap2DROS globalcostmap("global_costmap", tf);


  //crear el "action client"
  // true hace que el cliente haga "spin" en su propia hebra
  //Ojo: "move_base" puede ser cualquier string, pero hay que pensarlo como un "published topic". Por tanto, es importante que
  // el action server con el que va a comunicar tenga el mismo topic. E.d., al llamar a "as(n,string,....)" en el lado del
  // action server hay que usar el mismo string que en el cliente.
  MoveBaseClient ac("mi_move_base", true);  //<- poner "mi_move_base" para hacer mi propio action server.

  //creamos una hebra para que ros::spin se haga en paralelo y no interrumpa el procesamiento del cliente.
  //ros::spin es necesario para que los costmaps se actualicen adecuadamente.
  boost::thread spin_thread(&spinThread);

  //Esperar 60 sg. a que el action server esté levantado.
  ROS_INFO("Esperando a que el action server move_base se levante");
  //si no se conecta, a los 60 sg. se mata el nodo.
  ac.waitForServer(ros::Duration(60));

  ROS_INFO("Conectado al servidor mi_move_base");

  //Arrancar los costmaps una vez que estamos conectados al servidor
  localcostmap->start();
  globalcostmap->start();
  printCostmap(localcostmap);
  costmap_2d::Costmap2D *micostmap = localcostmap->getCostmap();
  std::vector<unsigned int> vecinas = findFreeNeighborCell(1096,micostmap);

  //ros::spin();

  move_base_msgs::MoveBaseGoal goal;
  vector<move_base_msgs::MoveBaseGoal> goals;

  goal.target_pose.header.frame_id = 	"map";
  goal.target_pose.header.stamp =	ros::Time::now();

  goal.target_pose.pose.position.x =	0.00;//-18.174;
  goal.target_pose.pose.position.y =	0.00;//25.876;
  goal.target_pose.pose.orientation.w =	1;

  goal_x = 0.00;
  goal_y = 0.00;

  ROS_INFO("Enviando el objetivo");

  goals.push_back(goal); //Definir como global

  while(!goals.empty()){
    ac.sendGoal(goals.back(),&doneCBGoal0,&activeCBGoal0,&feedbackCBGoal0);

    bool finalizado = false;

    ROS_WARN("Antes del while");
    while(!finalizado && !condicion_parada){
      ROS_INFO("La variable finalizado vale %d", finalizado);
      finalizado = ac.waitForResult(ros::Duration(1.0));
    }
    ROS_WARN("Después del while");

    if (condicion_parada){
      ROS_WARN("Entramos por condición de parada");
      if(goals.size()>1){
        ROS_WARN("Se ha fallado por alguna razón el segundo objetivo.");
        ros::shutdown();
        spin_thread.join();
        return 0;
      }
      ac.cancelGoal();
      ROS_WARN("Hemos cancelado el goal, pasamos a esperar 10 segundos");
      ac.waitForResult(ros::Duration(10.0));
      ROS_WARN("YA HEMOS TERMINADO DE ESPERAR");
      if (ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        ROS_INFO("Objetivo cancelado");

        geometry_msgs::PoseStamped start, goal;
        vector<geometry_msgs::PoseStamped> plan;

        rellenaPoseStamped (0, 0, start, localcostmap);
        rellenaPoseStamped (goals.back().target_pose.pose.position.x - prev_pos_x, goals.back().target_pose.pose.position.y - prev_pos_y, goal, localcostmap);
        //planificar nuevos goals
        localPlanner(start,goal, plan, localcostmap);
	      condicion_parada = false;
        tiempo_parado = 0;
        for (int i = 0; i < plan.size(); i++){
          move_base_msgs::MoveBaseGoal goal;

          goal.target_pose.header.frame_id = 	"map";
          goal.target_pose.header.stamp =	ros::Time::now();

          goal.target_pose.pose.position.x = -plan.at(i).pose.position.x + prev_pos_x;
          goal.target_pose.pose.position.y = plan.at(i).pose.position.y + prev_pos_y;
          goal.target_pose.pose.orientation.w =	1;
	        ROS_INFO("Vamos a insertar la sol (%f, %f)", goal.target_pose.pose.position.x,goal.target_pose.pose.position.y);
          goals.push_back(goal);
        }
      }
      else{
        ROS_INFO("Se ha fallado por alguna razón al cancelar.");
        ros::shutdown();
        spin_thread.join();
        return 0;
      }
    }
    else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("¡¡ Objetivo intermedio alcanzado !!");
      goals.pop_back();
    }
    else{
      ROS_INFO("Se ha fallado por alguna razón.");
      ros::shutdown();
      spin_thread.join();
      return 0;
    }
  }

  ROS_INFO("¡¡ Objetivo final alcanzado !!");
  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();


  return 0;

}
