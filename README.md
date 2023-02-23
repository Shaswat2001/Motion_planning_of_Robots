# Motion_planning_of_Robots

## Overview
This repository implements different motion planning algorithms like Heuristic and Incremental Search based and Sampling based algorithms in a 2D environment for a point robot. All the algorithms are implemented in python. The environment is implemented using a priority queue and is made effecient using the concept of OOPs.

## Project Structure

```
    .
    ├── Search Based            
    │   ├── Heuristic Search                
    │   │   ├── A star       
    │   │   ├── Bidirectional A star       
    │   │   ├── Dijkstra       
    │   │   ├── BFS      
    │   │   └── Anytime Repairing A star      
    |   |
    │   └── Incremental Search                
    │       ├── D star       
    │       ├── Lifelong Planning A star       
    │       ├── Dijkstra          
    │       └── Anytime Repairing A star 
    │ 
    └── Sampling Based
        ├── RRT       
        ├── Extend RRT       
        ├── Dynamic RRT       
        ├── RRT Connect      
        └── RRT star  

```

## Heuristic Search Based Algorithm

<div align=left>
<table>
  <tr>
    <td><img src="./Results/Astar.gif" alt="astar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/Bi-Astar.gif" alt="biastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/RepeatedA_star.gif" alt="repeatedastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/ARA_star.gif" alt="arastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/LRTA_star.gif" alt="lrtastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/RTAA_star.gif" alt="rtaastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/D_star.gif" alt="lpastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/LPAstar.gif" alt="dstarlite" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/ADstar_small.gif" alt="lpastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/ADstar_sig.gif" alt="dstarlite" width="400"/></a></td>
  </tr>
</table>
</div>

## Incremental Search Based Algorithm

## Contact

If you have any questions, please let me know:

- Shaswat Garg {[sis_shaswat@outlook.com]()}

