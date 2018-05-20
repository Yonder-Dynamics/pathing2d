# Pathing2D
Pathing2D is a pathfinding algorithm that takes an octomap as an input, converts it to a 2D map, and performs an A* search to find a path to the goal. It takes into account the height changes and optimizes the route both over distance and terrain difficulty. 

# PCL Locate bug fix  
If you are getting an issue with PCL not being found, try this:  
~~~~~~~~
sudo updatedb  
locate PCLConfig
~~~~~~~~
Copy the directory containing that file and run:   
`catkin_make -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl/  `

# Todo 
- Add orientation to each position
