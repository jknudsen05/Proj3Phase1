READ ME for Project3 Phase 1

Link For Videos: https://drive.google.com/drive/folders/133_b1Q2k_wsJVmb3v4zMU5bJZeEKDtQK?usp=share_link

Team:
Jens R Knudsen III -- UID 119482834
Pritom Raymond Leo Gomes -- UID 118428321

NOTE to TA:
Since the map is symetrical, I just flip the map when plotting it with a simple y transform

Libraries used in this code:
python "copy" library is loaded, but not used
python opencv2 library is loaded for visualization
python numpy library is loaded for array manipulation and plotting
python math library is loaded for trig functions

Functions defined in this code:
move_Left60 - 60 degree counter clockwise movement
move_Left30 - 30 degree counter clockwise movement
move_Straight - Applies straight forward move
move_Right30 - 30 degree clockwise movement
move_Right60 - 60 degree clockwise movement
generate_path - accepts reverse_path variable (which at this point is only the goal node 
		index and the parent node index) and searchs sequentially for parent nodes
		until the start node is reached.  It nows the start node has been reached
		when the next parent node is "N/A".  The function then uses a loop to
		reverse the order.  Then the meaningless parent node, "N/A", is removed
		from the list.  The forward path is then returned to the main code.
check_for_goal - checks if last popped node is withing 1.5 times the step size of the goal node
obstacle_check - used to check user inputs to see if they are valid

Instructions for use:
The only thing a user needs to do to run the program and click run.
They will be prompted to enter the radius of the robot and the step size of each move.
They will be then be shown the map and obstacles based on robot size, which the user 
should open the window and press any key to clear.
They will then be prompted to input a starting x coordinate after which they should press enter.
Then they will then be prompted to input a starting y coordinate after which they should press enter.
They will then be prompted to enter the robot's starting angle after which they should press enter.
The program will check to see if it the combination of x and y coordinates is a valid starting
position and if the angle entered is a valid entry.  If not, the program will promt for new entries.
This above process is followed to get the goal position.
Once the goal value is determined to be valid, the program will begin creating the node graph.
When the optimal path is found, a window will appear showing the order in which the nodes were
investigated.  Once the investigated nodes are done populating, the optimal path is shown in green.



