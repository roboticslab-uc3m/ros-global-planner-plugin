/*  
 * University Carlos III of Madrid, Spain.
 * All rights reserved.
 *        Files: roboticslab_plan_node.hpp
 *   Created on: 20, 09, 2022 
 *       Author: Ainhoa de Matias 
 *        Email: amatias@pa.uc3m.es
 */

/*
 * Last update.
 *     Date: 20, 09, 2022 
 *   Author: Ainhoa de Matias
 *    Email: amatias@pa.uc3m.es
 */

#ifndef ROBOTICSLAB_PLAN_NODE
#define ROBOTICSLAB_PLAN_NODE

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cv_bridge/cv_bridge.h>

// ## A nivel mapa
// ### Del mapa original
// ### * 0: libre
// ### * 1: ocupado 
// ### * 2: visitado
// ### * 3: start
// ### * 4: goal


// ### * -2: parentId del nodo start
// ### * -1: parentId del nodo goal PROVISIONAL cuando aun no se ha resuelto

// ## Initial values are hard-coded (A nivel mapa)

namespace roboticslab{

class Node
{   
    // private:
    //     int _x,_y,_id,_parentId;

    public:
        Node(int x,int y, int z, int id,int parentId) : _x(x), _y(y), _z(z),_id(id), _parentId(parentId) {}
        int _x,_y, _z, _id,_parentId;
};

};



#endif // ROBOTICSLAB_PLAN_NODE
