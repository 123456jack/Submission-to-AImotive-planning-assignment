# Dependencies
The submitted solution employs a Model Predictive Controller (MPC). The current implementation of the MPC controller relies on the IPOPT solver (Interior Point OPTimizer) and the CppAD (C++ Algorithmic Differentiation) package for solving the optimization problem and calculating the gradients, respectively.

## Intalling Ipopt and CppAD

1.  Clone this repository and navigate to the cloned directory
2.  [Download](https://www.coin-or.org/download/source/Ipopt/) the appropriate version of Ipopt (3.12.7 or higher) from the link below.  You may also use wget or a similiar command to download the source from the command line (see Linux instructions).
3.  Follow the instructions for your environment

* [Ipopt](https://projects.coin-or.org/Ipopt)
  * **Mac:**
    ```
      brew tap brewsci/science
      brew install ipopt --with-openblas
    ```
  * **Linux:**
    * ```sudo apt-get install gfortran```
    *  ```apt-get install unzip```
    * ```wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip```
    * Call `install_ipopt.sh` with the source directory as the first argument, ex: ```./install_ipopt.sh Ipopt-3.12.7``` or ```bash install_ipopt.sh Ipopt-3.12.7```


* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent. You can also compile and install from source by following the instructions given at [CppAD/install](https://www.coin-or.org/CppAD/Doc/install.htm).


## Gazebo setup instructions 

### Linux 

Gazebo can be most easily ran on Linux (Debian/Ubuntu is the preferred distribution).  

 - Please follow the setup instructions on the Gazebo website (http://gazebosim.org/tutorials?tut=install_ubuntu) 

# AImotive planning assiginment setup instructions 

## Linux

 - Clone this repository 

 - Change to the world directory and run gazebo 
```
        cd /path/to/this/dir/world
        export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:`pwd`
        gazebo --verbose interview_city.world
```

- (Optional) run the following command for a faster simulation, but without visualization.

	```gzserver --verbose interview_city.world```

 

 - Compile and run the client 
 ```
        cd build
        cmake ..
        make 
        ./client_controller
```

### Windows/Mac/other Linux distributions

Gazebo server can be ran on any platform supporting Docker. In this case you will not see the visualisation
provided by Gazebo's client interface, but the simulation itself is fully functional.

 - Install Docker on your operating system
    - https://www.docker.com/get-started
 - Change to the directory of this repository
 - Build the container
    - `docker build -t gazebo_assignment_app .`
 - Run the container
    - `docker run -it --name gazebo_assignment_container -p 11345:11345 -v "/PATH/TO/A/LOCAL/DIRECTORY:/root/.gazebo/" gazebo_assignment_app`
    - It is recommended to mount a local directory into the container to store temporary GAzebo files, as this way
    the model files used by the simulation are only downloaded once. Change `/PATH/TO/A/LOCAL/DIRECTORY` to a 
    valid path on your file system in the command above.
    - Starting the simulator for the first time takes a few minutes, as it is downloading model files. Please be patient.
 - You can now connect to the simulator on localhost:11345. Optionally compile and run the sample client (see the code above)
 
Please check out the Gazebo Docker tutorial for more information on using this setup: https://hub.docker.com/_/gazebo/ 
