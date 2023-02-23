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
    <td><img src="./Results/Dijkstra.gif" alt="Dijkstra" width="450"/></a></td>
    <td><img src="./Results/Astar.gif" alt="Astar" width="450"/></a></td>
  </tr>
</table>

<table>
  <tr>
    <td><img src="./Results/BiAstar.gif" alt="biastar" width="450"/></a></td>
    <td><img src="./Results/ARA.gif" alt="AnytimeAstar" width="450"/></a></td>
  </tr>
</table>
</div>

## Incremental Search Based Algorithm

## Sample Based Algorithm

<div align=left>
<table>
  <tr>
    <td><img src="./Results/RRT.gif" alt="RRT" width="450"/></a></td>
    <td><img src="./Results/RRTConnect.gif" alt="RRTConnect" width="450"/></a></td>
  </tr>
</table>

<table>
  <tr>
    <td><img src="./Results/RRTStar.gif" alt="RRTStar" width="450"/></a></td>
    <td><img src="./Results/RRTExtend.gif" alt="RRTExtend" width="450"/></a></td>
  </tr>
</table>
</div>

## Contact

If you have any questions, please let me know:

- Shaswat Garg {[sis_shaswat@outlook.com]()}

