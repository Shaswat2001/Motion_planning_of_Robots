# Motion Planning Algorithms for Robots

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
    │       └── Anytime Repairing A star 
    │ 
    └── Sampling Based
        ├── Multi Query Search                
        │   ├── RRT       
        │   ├── Extend RRT       
        │   ├── Dynamic RRT       
        │   ├── RRT Connect    
        │   ├── FMT* 
        │   ├── RRT#
        │   ├── RRT*  
        │   ├── Informed RRT* 
        │   └── RRT* Smart      
        |
        └── Single Query Search                
            ├── PRM       
            └── LazyPRM       

```

## Heuristic Search Based Algorithm

<div align=left>
<table>
  <tr>
    <td><img src="./Results/Dijkstra.gif" alt="Dijkstra" width="350"/></a></td>
    <td><img src="./Results/Astar.gif" alt="Astar" width="350"/></a></td>
  </tr>
</table>

<table>
  <tr>
    <td><img src="./Results/BiAstar.gif" alt="biastar" width="350"/></a></td>
    <td><img src="./Results/ARA.gif" alt="AnytimeAstar" width="350"/></a></td>
  </tr>
</table>
</div>

## Incremental Search Based Algorithm

<div align=left>
<table>
  <tr>
    <td><img src="./Results/RTAAstar.gif" alt="RTAAstar" width="350"/></a></td>
    <td><img src="./Results/LPAstar.gif" alt="LPAstar" width="350"/></a></td>
  </tr>
</table>

<table>
  <tr>
    <td><img src="./Results/Dstar.gif" alt="Dstar" width="350"/></a></td>
  </tr>
</table>
</div>

## Multi Query Sample Based Algorithm

<div align=left>
<table>
  <tr>
    <td><img src="./Results/RRT.gif" alt="RRT" width="350"/></a></td>
    <td><img src="./Results/RRTConnect.gif" alt="RRTConnect" width="350"/></a></td>
  </tr>
</table>

<table>
  <tr>
    <td><img src="./Results/RRTStar.gif" alt="RRTStar" width="350"/></a></td>
    <td><img src="./Results/RRTExtend.gif" alt="RRTExtend" width="350"/></a></td>
  </tr>
</table>

<table>
  <tr>
    <td><img src="./Results/FMTStar.gif" alt="FMTStar" width="350"/></a></td>
    <td><img src="./Results/DynamicRRT.gif" alt="DynamicRRT" width="350"/></a></td>
  </tr>
</table>
</div>

## Single Query Sample Based Algorithm

<div align=left>
<table>
  <tr>
    <td><img src="./Results/PRM.gif" alt="PRM" width="350"/></a></td>
    <td><img src="./Results/LazyPRM.gif" alt="LazyPRM" width="350"/></a></td>
  </tr>
</table>
</div>

## Contact

If you have any questions, please let me know:

- Shaswat Garg {[sis_shaswat@outlook.com]()}

