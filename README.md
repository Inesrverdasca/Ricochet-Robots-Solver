# Ricochet_Robots_Solver
Implementation of a Ricochet Robots puzzle solver using A* search


Description:
-> Optimal solver for the Ricochet Robots puzzle
-> Recieves as an argument a file with the board size, walls, robots positions and target
-> Uses A* search to find the path between the start and end node
-> The output is the sequence of movements that will enable the required robot to reach the target in the fewest movements and the number of movements

Input File:
-> In the first line the board size is defined (NXN)
-> Then the next four lines correspond to the positions of the robots 
-> The fifth corresponds to the target node
-> The sixth the number of walls (n)
-> Then n lines defining the positioning of the walls "line collumn direction". Example "3 4 r"

Output:
-> Prints total sum of movements done
-> Prints the sequence of movements by order 
