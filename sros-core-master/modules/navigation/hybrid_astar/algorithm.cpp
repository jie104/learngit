/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-09-23 10:06:52
 * @LastEditTime: 2021-07-20 14:11:24
 * @LastEditors: Please set LastEditors
 */
#include "algorithm.h"
#include "dubins.h"
#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace,MapGrid& gridmap);

Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};


//###################################################
//                                        3D A*
//###################################################
Node3D*  Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,MapGrid& gridmap) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // VISUALIZATION DELAY
//  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  // update h value
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace,gridmap);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  // float max = 0.f;

  // continue until O empty
  while (!O.empty()) {

    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;
    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }// EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      //如果nPred->getX() ，nPred->getY() 如果遍历到的点周围允许原地旋转，则停止搜索。
      if(configurationSpace.isRotateTraversable(nPred)){
        LOG(INFO)<<"搜索到可以原地旋转的目标点！！！";
        return  nPred;
      }
      // GOAL TEST
      if (hypot(nPred->getX() - goal.getX(), nPred->getY() - goal.getY()) < 20.0 / configurationSpace.cfg_.nav_resolution || 
          iterations > Constants::iterations) {
        // DEBUG

        if( iterations > Constants::iterations){
            LOG(INFO)<<"hybrid a star 遍历超过"<<Constants::iterations<<"次都没有找到合适点 ";
          return nPred;
        }
        LOG(INFO)<<"hybrid a star 遍历的次数为"<< iterations<<"时，接近目标点，但未找到可以原地旋转的点。";
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          LOG(INFO)<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!开始搜索dubin路径。"<<goal.getX()<<":"<<goal.getY()<<";"<<goal.getT();
          nSucc = dubinsShot(*nPred, goal, configurationSpace);

          if (nSucc != nullptr) {
            //DEBUG
             LOG(INFO) << "xxxxxxxxx得到最后的路径"<<nSucc->getX()<<";"<<nSucc->getY()<<";"<<nSucc->getT();
            return nSucc;
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {
           // LOG(INFO)<<"111111111111111：";
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);

          // ensure successor is on grid and traversable

          // 判断该扩展点是否碰到障碍物，之前采用判断 该点是否在障碍物中，现在改为判断整个AGV内是否有障碍物。避免出现后退
          // 转到前进时，扩展点不在障碍物中，但是agv中却又障碍物。
          if (nSucc->isOnGrid(width, height) && configurationSpace.check_line_collision(nSucc)) {

           //   std::cout<<"2222222"<<std::endl;
              bool isclosed = !nodes3D[iSucc].isClosed();
            //  LOG(INFO)<<"被扩展的点是否在close集合中"<<isclosed;
            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {
              //  LOG(INFO)<<"333333333："<< iSucc;
              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {
                //  LOG(INFO)<<"444444444444："<< iSucc;
                // calculate H value
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace,gridmap);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                 // LOG(INFO)<<"22222222222："<< iSucc;
                // 每次放入o时都打印一次
              //  visual->publishHybridSearchPoint(*nSucc,*nPred,i);
                 // usleep(1000);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
      LOG(INFO)<<"OPEN SET IS  EMPTY!!!!!";
    return nullptr;
  }

  LOG(INFO) << "hybird A* failed to find a valid path.";

  return nullptr;
}

/** @brief 从起点开始，寻找一个可以原地旋转的虚拟目标点
 *  @param start 规划的起点
 *  @param goal 最终得到的虚拟目标点
 *  @param nodes3D hybrid A*的扩展点
 *  @param nodes2D 暂无实际意义
 *  @param width 局部地图的宽度
 *  @param heigth 局部地图高度
 *  @param configurationSpace 全局碰撞检测地图
 *  @param dubinsLookup hybrid A* 的扩展步长
 *  @param gridmap 局部地图
**/
Node3D*  Algorithm::findOneRotatePointForAgv(Node3D& start,
                                            const Node3D& goal,
                                            Node3D* nodes3D,
                                            Node2D* nodes2D,
                                            int width,
                                            int height,
                                            CollisionDetection& configurationSpace,
                                            float* dubinsLookup,MapGrid& gridmap) {
    // PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    float newG;
    // Number of possible directions, 3 for forward driving and an additional 3 for reversing
    int dir = Constants::reverse ? 6 : 3;
    // Number of iterations the algorithm has run for stopping based on Constants::iterations
    int iterations = 0;

    // OPEN LIST AS BOOST IMPLEMENTATION
    typedef boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes> > priorityQueue;
    priorityQueue O;

    // update h value
    updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace,gridmap);
    // mark start as open
    start.open();
    // push on priority queue aka open list
    O.push(&start);
    iPred = start.setIdx(width, height);
    nodes3D[iPred] = start;

    // NODE POINTER
    Node3D* nPred;
    Node3D* nSucc;

    // continue until O empty
    while (!O.empty()) {
        // pop node with lowest cost from priority queue
        nPred = O.top();
        // set index
        iPred = nPred->setIdx(width, height);
        iterations++;
        // LAZY DELETION of rewired node
        // if there exists a pointer this node has already been expanded
        if (nodes3D[iPred].isClosed()) {
            // pop node from the open list and start with a fresh node
            O.pop();
            continue;
        }// EXPANSION OF NODE
        else if (nodes3D[iPred].isOpen()) {
            // add node to closed list
            nodes3D[iPred].close();
            // remove node from open list
            O.pop();

            //如果nPred->getX() ，nPred->getY() 如果遍历到的点周围允许原地旋转，则停止搜索。
            if(configurationSpace.isRotateTraversable(nPred)){
                LOG(INFO)<<"搜索到可以原地旋转的目标点！！！";
                return  nPred;
            }
            // GOAL TEST
            if (hypot(nPred->getX() - goal.getX(), nPred->getY() - goal.getY()) < 20.0 / configurationSpace.cfg_.nav_resolution || 
                iterations > Constants::iterations) {
                // DEBUG

                // if( iterations > Constants::iterations){
                //     LOG(INFO)<<"hybrid a star 遍历超过"<<Constants::iterations<<"次都没有找到合适点 ";
                //     return nPred;
                // }
                // LOG(INFO)<<"hybrid a star 遍历的次数为"<< iterations<<"时，接近目标点，但未找到可以原地旋转的点。";
                // return nPred;
            }

            // ____________________
            // CONTINUE WITH SEARCH
            else {
                // _______________________
                // SEARCH WITH DUBINS SHOT
                if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
                    LOG(INFO)<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!开始搜索dubin路径。"<<goal.getX()<<":"<<goal.getY()<<";"<<goal.getT();
                    nSucc = dubinsShot(*nPred, goal, configurationSpace);

                    if (nSucc != nullptr) {
                        //DEBUG
                        LOG(INFO) << "xxxxxxxxx得到最后的路径"<<nSucc->getX()<<";"<<nSucc->getY()<<";"<<nSucc->getT();
                        return nSucc;
                    }
                }

                // SEARCH WITH FORWARD SIMULATION
                for (int i = 0; i < dir; i++) {
                    // LOG(INFO)<<"111111111111111：";
                    // create possible successor
                    nSucc = nPred->createSuccessor(i);
                    // set index of the successor
                    iSucc = nSucc->setIdx(width, height);

                    // ensure successor is on grid and traversable

                    // 判断该扩展点是否碰到障碍物，之前采用判断 该点是否在障碍物中，现在改为判断整个AGV内是否有障碍物。避免出现后退
                    // 转到前进时，扩展点不在障碍物中，但是agv中却又障碍物。
                    if (nSucc->isOnGrid(width, height) && configurationSpace.check_line_collision(nSucc)) {
                        //   std::cout<<"2222222"<<std::endl;
                        bool isclosed = !nodes3D[iSucc].isClosed();
                        //  LOG(INFO)<<"被扩展的点是否在close集合中"<<isclosed;
                        // ensure successor is not on closed list or it has the same index as the predecessor
                        if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {
                            //  LOG(INFO)<<"333333333："<< iSucc;
                            // calculate new G value
                            nSucc->updateG();
                            newG = nSucc->getG();

                            // if successor not on open list or found a shorter way to the cell
                            if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {
                                //  LOG(INFO)<<"444444444444："<< iSucc;
                                // calculate H value
                                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace,gridmap);

                                // if the successor is in the same cell but the C value is larger
                                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                                    delete nSucc;
                                    continue;
                                }
                                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                                    nSucc->setPred(nPred->getPred());
                                }

                                if (nSucc->getPred() == nSucc) {
                                    std::cout << "looping";
                                }

                                // put successor on open list
                                nSucc->open();
                                nodes3D[iSucc] = *nSucc;
                                O.push(&nodes3D[iSucc]);
                                // LOG(INFO)<<"22222222222："<< iSucc;
                                // 每次放入o时都打印一次
                                //  visual->publishHybridSearchPoint(*nSucc,*nPred,i);
                                // usleep(1000);
                                delete nSucc;
                            } 
                            else { delete nSucc; }
                        }
                        else { delete nSucc; }
                    }
                    else { delete nSucc; }
                }
            }
        }
    }

    if (O.empty()) {
        LOG(INFO)<<"OPEN SET IS  EMPTY!!!!!";
        return nullptr;
    }

    LOG(INFO) << "hybird A* failed to find a valid path.";

    return nullptr;
}


//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  //ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
//        visualization.publishNode2DPoses(*nPred);
//        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace,MapGrid& gridmap) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  double dx = start.getX() - goal.getX();
  double dy = start.getY() - goal.getY();

  ReedsSheppStateSpace rs_space;
 // ReedsSheppStateSpace::ReedsSheppPath path = rs_space.reedsShepp(start,goal);
  twoDCost = gridmap.map_[gridmap.getIndex(start.getX(),start.getY())].target_dist;

  //twoDCost = sqrt(dx*dx+dy*dy);

  //LOG(INFO)<<"twoDCost："<<twoDCost;

//  ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
//  State* rsStart = (State*)reedsSheppPath.allocState();
//  State* rsEnd = (State*)reedsSheppPath.allocState();
//  rsStart->setXY(start.getX(), start.getY());
//  rsStart->setYaw(start.getT());
//  rsEnd->setXY(goal.getX(), goal.getY());
//  rsEnd->setYaw(goal.getT());
//  reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
  // return the maximum of the heuristics, making the heuristic admissable
  //std::cout<<"RR路径长度："<<dubinsCost<<std::endl;

//  Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
//  // create a 2d goal node
//  Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
//  float aStarLength = aStar(goal2d, start2d, nodes2D, width, height, configurationSpace);
//  twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
//                    ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
//  aStarLength = aStarLength + twoDoffset;
//  std::cout<<"aStarLength："<<aStarLength<<std::endl;

  start.setH(1.1*twoDCost);
}


void updateH1(Node3D& start, const Node3D& goal) {
  float twoDCost = 0;

  double dx = start.getX() - goal.getX();
  double dy = start.getY() - goal.getY();

  twoDCost = sqrt(dx*dx+dy*dy);

  start.setH(1.5*twoDCost);
}

//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);
//
  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);
  std::cout<<"dubins length: "<<length;
  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];
  double q[3];
  dubins_path_sample(&path,length, q);
  std::cout<<"zhogndian :"<<q[0]<<";"<<q[1]<<";"<<q[2]<<std::endl;
  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (true) {
      std::cout << "如果没有障碍物"<<std::endl;
      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

   std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}
