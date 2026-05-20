Usage

0) Prerequisite
build: cmake, boost, and sparsehash (https://github.com/sparsehash/sparsehash)

1) Build & Run

	0. enter the folder of ALGORITHM:
		``cd ./ALGORITHM/''

	1. change the 'driver.cpp' to specify the path of your input and output:
	   map_file & task_file: input of the MAPD instance, which are stored in './Instances/'
	   out_file: specify the path to store the log file
	   tour_file (for TA algorithms only): calculated task sequences from LKH3, which are stored in './tour'
	   tsp_file & par_file (for TA algorithms only): specify the path to store the input of LKH3, TA-algorithms will later generate these files automatically, you can also change the 'Simulation.cpp' to modify the parameters (for example, to change the time limit or iteration limit for LKH3).
	   don't forget to call simu.run in 'driver.cpp'

	2. build the driver and the simulator:
	   ``cmake .''
	   ``make .''

	3. run the driver:
		``.\driver'' 
		and results are printed on the screen

2) If you want to re-calculate the task sequences (for TA algorithms only) using LKH3:
	
	0. build LKH3: 
	   ``cd ./LKH3/''
	   ``make .''
	   note: we have modified the code of LKH3, which differs from the release version on the website

	1. follow the way in 1) to generate the par_file & tsp_file

	2. ``./LKH3/LKH $par_file''

	3. LKH3 will store the task sequences at the path specified in par_file



# mapd-lns
