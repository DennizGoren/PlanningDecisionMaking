# Planning and Decision Making Project

<p float="left">
  <img src="/Visualisations/scene/corner.png" height="350" width="auto" />
  
</p>

## Authors

Aden Westmaas (4825373)  
Badr Essabri (5099412)     
Denniz Goren (4495543)    
Renée Schwietert (4551753)    

## Requirements
matplotlib==3.6.2  
numpy==1.24.1  
scipy==1.10.0  
tqdm==4.64.1  
[CoppeliaSim](https://www.coppeliarobotics.com/downloads)  

## Running the software

1. Download [CoppeliaSim](https://www.coppeliarobotics.com/downloads)  
2. Clone the repository  
    ```sh
    git clone https://github.com/DennizGoren/PlanningDecisionMaking.git
    ```  
3. Install required packages
   ```sh
   pip install -r requirements.txt
   ```
4. Set up a ```PYTHONPATH``` environment variable to link the CoppeliaSim API to the repository
    ```sh
   export PYTHONPATH=~/Applications/coppeliaSim.app/Contents/Resources/programming/zmqRemoteApi/clients/python
   ```
5. Load the ```scene.ttt``` file into the CoppeliaSim environment by going to File -> Open scene...
6. Make sure that there is not an active simulation in CoppeliaSim and run the following for RRT* to find a path and the simulation to start
    ```sh 
    python RobotControl.py 
    ```

## Results

<p float="left">
  <img src="/Visualisations/4000/path.gif" height="350" width="400" />
  <img src="/Visualisations/scene/top.png" height="350" width="350"/> 
</p>

<p float="left">
  <img src="/Visualisations/4000/path_4000.png" height="350" width="400" />
  <img src="/Visualisations/4000/simulation.gif" height="350" width="350" /> 
</p>

## Acknowledgments


