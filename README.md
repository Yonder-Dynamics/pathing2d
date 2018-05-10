# PCL Locate bug fix  
If you are getting an issue with PCL not being found, try this:  
~~~~~~~~
sudo updatedb  
locate PCLConfig
~~~~~~~~
Copy the directory containing that file and run:   
`catkin_make -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl/  `

# Todo 
- Link all nodes to the current position to make sure there is an entry point
- Finish shortest path algo
- Add orientation to each position
- Test :)
