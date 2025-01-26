# APRG - An Efficient Plane Segmentation Algorithm  

This is the open-source code for the **APRG (On Review)**.

## Dependencies

The code depends on the following libraries:
- [OpenCV](https://opencv.org/)
- [Eigen3](https://eigen.tuxfamily.org/)

## How to Set Up and Run

For a simple try-out, you can use the pre-built Docker image, which comes with some sample data and config for convenience. For developers, we recommend using **VS Code Dev Container** to create a development environment from the provided `Dockerfile` and start developing in container.

### (option 1) Use Pre-build Docker Image

   1. Pull the provided Docker image from the container registry:
      ```bash
      docker pull lantaozhang/aprg
      ```
   2. Run the Container:
      ```bash
      docker run -it --name aprg_sample lantaozhang/aprg
      ```
   3. Run sample data:
      ```bash
      ./build/Run_EVOPS ./config/config_sample.json
      ```
   4. Exit the container, copy the sample data to the host machine, and check out the output in your ~/sample_data folder.
      ```bash
      docker cp aprg_sample:/APRG/data/sample_data ~
      ```
### (option 2) Set Up the Development Environment

1. **Install Prerequisites**:
   - Install [Docker](https://www.docker.com/).
   - Install [Visual Studio Code](https://code.visualstudio.com/).
   - Install the [VS Code Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

2. **Build and Launch the Dev Container**:
   - Open the project folder in VS Code.
   - VS Code will detect the `Dockerfile` and suggest reopening in a Dev Container. Follow the prompts to build and open the container.
   - To test EVOPS benchmark, edit the `devcontainer.json` file (located under `.devcontainer/devcontainer.json`) to mount your data folder like the following:  
     ```json  
     "mounts": [  
         {  
             "type": "bind",  
             "source": "<your dataset path>",  
             "target": "/workspaces/APRG/data/EVOPS"  
         }  
     ]  
     ```  
3. **Ensure Configuration Matches Your Dataset**:
   - Ensure the settings in `config/config_sample.json` match your specific dataset.
   
4. **Compile and Run in the Container**:
   - Once inside the container's interactive terminal, compile and run the code as follows:
     ```bash
     mkdir build && cd build
     cmake ..
     make
     cd ..
     ./build/Run_EVOPS ./config/config_sample.json
     ```

## Notes
- Adjust the `Dockerfile` if additional dependencies are required.

Feel free to contribute to this repository by submitting issues or creating pull requests!
