# FailTeamLab3

This repository is maintained by the Fail Team, which includes Abdirakhman Onabek, Daniil Filimonov, and Kir Smolyarchuk. It is structured as a ROS (Robot Operating System) workspace.

## Project Overview

The project aims to develop and test various robotics-related software components using ROS.

### Repository Structure

- **catkin_ws/**: Contains ROS workspace files, including build, devel, and source directories.
- **scripts/**: This directory includes Python scripts used for various robotic functionalities:
  - **DexterousSpace.py**: Implements calculations for dexterous space.
  - **IK.py**: Contains the implementation for inverse kinematics.
  - **PolyTraj.py**: Manages polynomial trajectory generation.
- **requirements.txt**: Lists all Python dependencies needed for this project.

### Getting Started

1. **Clone the Repository**:
   Clone the repository to your local machine using the following command:
   ```bash
   git clone https://github.com/Meneralllka/FailTeamLab3.git
   ```
   
2. **Build the Workspace**:
   Navigate into the cloned directory and build the ROS workspace:
   ```bash
   cd FailTeamLab3/catkin_ws
   catkin_make
   ```
   
3. **Set Up the Environment**:
   Source the setup file to configure your environment for ROS:
   ```bash
   source devel/setup.bash
   ```
   
4. **Install Dependencies**:
   Install the required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

5. **Run ROS Nodes**:
   Launch your ROS nodes or test your packages using ROS commands:
   ```bash
   rosrun <package_name> <node_name>
   ```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Feel free to fork the repository, create a new branch, and submit a pull request.

## Contact

For any questions or issues, please reach out to the Fail Team members via GitHub.

