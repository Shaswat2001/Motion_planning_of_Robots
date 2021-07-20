from tkinter import *
from A_star import doA_star
from Dijkstra import doDijkstra
from PRM import doPRM_Algorithm
from RRT import doRRT
from BFS import doBFS

# Creating main window
m=Tk()
m.title('Motion Planning')
# Size of the window
m.geometry('300x300')

# Button for A_star Algorithm
Astar_bt = Button(m, text='A_star Algorithm', bd=10, command=doA_star)
# Button for Dijkstra Algorithm
dijk_bt = Button(m, text='Dijkstra Algorithm', bd=10, command=doDijkstra)
# Button for PRM Algorithm
PRM_bt = Button(m, text='PRM Algorithm', bd=10, command=doPRM_Algorithm)
# Button for RRT Algorithm
RRT_bt = Button(m, text='RRT Algorithm', bd=10, command=doRRT)
# Button for BFS Algorithm
BFS_bt = Button(m, text='BFS Algorithm', bd=10, command=doBFS)
# Exit Button
exit = Button(m, text='Exit', bd=10, command=m.destroy)

# Placing the button in the window
Astar_bt.pack(side='top')
dijk_bt.pack(side='top')
PRM_bt.pack(side='top')
RRT_bt.pack(side='top')
BFS_bt.pack(side='top')
exit.pack(side='bottom')

m.mainloop()
