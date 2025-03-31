## Catkin_ws

This README exists to aid in structuring the file system of VirT&R. This folder is where the catkin workspace for two key ros packages will be installed and made. Several files will need to added to subfolders in this directory to add meshes to Gazebo correctly. Helper functions will help with creating the .world and .launch files, but specific structuring to add the meshes is required. Once this directory has been made, inside the /warthog_simulator/warthog_gazebo/ folder, a folder called 'models' must be made and have the following structure:

models
├── <project_mesh_name>
│   ├── model.config
│   ├── model.sdf
│   └── meshes
│       ├── material_0.png
│       └── mesh.dae
├── <project_mesh_name>
│   ├── ...
└── <project_mesh_name>
    ├── ...


Warthog_simulator and warthog packages will be cloned and installed during the Docker buid and run phases, and the projects can be found here for further reference: 

https://github.com/warthog-cpr/warthog_simulator/tree/melodic-devel

https://github.com/warthog-cpr/warthog

## [License](./LICENSE)
