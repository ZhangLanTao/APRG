# APRG - An Efficient Plane Segmentation Algorithm  

This is the open-source code for the **APRG (On Review)**.

## Demo
These are our outputs on ICL and TUM dataset.

![GIF1](<media/ICL livingroom.gif>) ![GIF2](<media/ICL office.gif>) ![GIF3](<media/TUM desk.gif>)
![GIF4](<media/TUM long office.gif>) ![GIF5](<media/TUM pioneer.gif>) ![GIF6](<media/TUM cabinet.gif>)

## Results on [EVOPS](https://evops.netlify.app/) Benchmark
|     | Method            | Precision | Recall | USR  | OSR  | Missed | Noise | IoU  | DICE | Time(fps) | Mem(MB) |
| --- | ----------------- | --------- | ------ | ---- | ---- | ------ | ----- | ---- | ---- | --------- | :-----: |
| ICL | DDPFF             | 0.27      | 0.39   | 0.07 | 0.01 | 0.46   | 0.65  | 0.88 | 0.92 | 36.19     |    -    |
|     | CAPE              | 0.59      | 0.49   | 0.08 | 0.03 | 0.39   | 0.26  | 0.93 | 0.96 | 370.51    |         |
|     | PlaneRecNet       | 0.66      | 0.44   | 0.07 | 0.01 | 0.49   | 0.26  | 0.91 | 0.95 | 38.25     |         |
|     | SAM               | 0.16      | 0.43   | 0.04 | 0.07 | 0.35   | 0.76  | 0.92 | 0.96 | 0.68      |         |
|     | GroundingDINO+SAM | 0.60      | 0.35   | 0.06 | 0.00 | 0.60   | 0.34  | 0.95 | 0.97 | 2.30      |         |
|     | APRG(ours)        | 0.66      | 0.59   | 0.06 | 0.02 | 0.30   | 0.25  | 0.95 | 0.98 | 620.05    |         |
| TUM | DDPFF             | 0.03      | 0.51   | 0.00 | 0.08 | 0.38   | 0.96  | 0.71 | 0.77 | 48.97     |   32    |
|     | CAPE              | 0.28      | 0.30   | 0.00 | 0.06 | 0.64   | 0.63  | 0.54 | 0.58 | 495.98    |   119   |
|     | PlaneRecNet       | 0.40      | 0.48   | 0.01 | 0.01 | 0.50   | 0.58  | 0.66 | 0.72 | 29.04     |  4183   |
|     | SAM               | 0.09      | 0.43   | 0.01 | 0.07 | 0.47   | 0.89  | 0.64 | 0.69 | 0.71      |  6595   |
|     | GroundingDINO+SAM | 0.30      | 0.36   | 0.06 | 0.00 | 0.57   | 0.63  | 0.58 | 0.63 | 1.79      |  15207  |
|     | APRG(ours)        | 0.54      | 0.61   | 0.00 | 0.07 | 0.31   | 0.38  | 0.85 | 0.90 | 724.69    |   62    |

<details>
<summary><strong>Click to show detailed results.</strong></summary>

|                 | Method            | Precision | Recall | USR  | OSR  | Missed | Noise | IoU  | DICE | Time(fps) |
| --------------- | ----------------- | --------- | ------ | ---- | ---- | ------ | ----- | ---- | ---- | --------- |
| ICL living room | DDPFF             | 0.25      | 0.33   | 0.10 | 0.01 | 0.47   | 0.64  | 0.85 | 0.89 | 37.24     |
|                 | CAPE              | 0.58      | 0.50   | 0.10 | 0.01 | 0.39   | 0.30  | 0.93 | 0.96 | 366.68    |
|                 | PlaneRecNet       | 0.58      | 0.50   | 0.10 | 0.01 | 0.39   | 0.30  | 0.93 | 0.96 | 37.97     |
|                 | SAM               | 0.19      | 0.46   | 0.05 | 0.04 | 0.32   | 0.73  | 0.94 | 0.97 | 0.66      |
|                 | GroundingDINO+SAM | 0.63      | 0.33   | 0.07 | 0.00 | 0.63   | 0.30  | 0.96 | 0.98 | 2.32      |
|                 | ours              | 0.50      | 0.54   | 0.08 | 0.02 | 0.32   | 0.40  | 0.95 | 0.98 | 623.36    |
| ICL office      | DDPFF             | 0.29      | 0.46   | 0.03 | 0.01 | 0.45   | 0.67  | 0.92 | 0.96 | 35.13     |
|                 | CAPE              | 0.61      | 0.49   | 0.06 | 0.05 | 0.39   | 0.22  | 0.93 | 0.96 | 374.33    |
|                 | PlaneRecNet       | 0.74      | 0.37   | 0.04 | 0.00 | 0.59   | 0.22  | 0.89 | 0.94 | 38.53     |
|                 | SAM               | 0.12      | 0.41   | 0.02 | 0.11 | 0.39   | 0.79  | 0.91 | 0.95 | 0.70      |
|                 | GroundingDINO+SAM | 0.57      | 0.38   | 0.05 | 0.00 | 0.57   | 0.38  | 0.93 | 0.96 | 2.28      |
|                 | ours              | 0.83      | 0.65   | 0.04 | 0.02 | 0.28   | 0.09  | 0.96 | 0.98 | 616.74    |
| TUM desk        | DDPFF             | 0.01      | 0.15   | 0.00 | 0.12 | 0.69   | 0.97  | 0.41 | 0.45 | 38.77     |
|                 | CAPE              | 0.16      | 0.23   | 0.00 | 0.08 | 0.69   | 0.75  | 0.64 | 0.70 | 414.57    |
|                 | PlaneRecNet       | 0.09      | 0.11   | 0.01 | 0.01 | 0.86   | 0.90  | 0.28 | 0.31 | 28.43     |
|                 | SAM               | 0.01      | 0.06   | 0.00 | 0.04 | 0.89   | 0.98  | 0.18 | 0.20 | 0.77      |
|                 | GroundingDINO+SAM | 0.06      | 0.04   | 0.03 | 0.01 | 0.86   | 0.91  | 0.16 | 0.17 | 1.76      |
|                 | ours              | 0.19      | 0.32   | 0.00 | 0.06 | 0.62   | 0.74  | 0.76 | 0.81 | 547.44    |
| TUM long office | DDPFF             | 0.03      | 0.33   | 0.00 | 0.14 | 0.51   | 0.95  | 0.73 | 0.78 | 38.31     |
|                 | CAPE              | 0.28      | 0.33   | 0.00 | 0.10 | 0.57   | 0.55  | 0.74 | 0.79 | 404.06    |
|                 | PlaneRecNet       | 0.27      | 0.29   | 0.01 | 0.03 | 0.67   | 0.68  | 0.72 | 0.78 | 28.20     |
|                 | SAM               | 0.03      | 0.22   | 0.00 | 0.07 | 0.68   | 0.95  | 0.66 | 0.70 | 0.75      |
|                 | GroundingDINO+SAM | 0.17      | 0.11   | 0.06 | 0.01 | 0.83   | 0.75  | 0.38 | 0.41 | 1.80      |
|                 | ours              | 0.24      | 0.38   | 0.00 | 0.14 | 0.48   | 0.61  | 0.81 | 0.87 | 526.67    |
| TUM pioneer     | DDPFF             | 0.02      | 0.70   | 0.00 | 0.04 | 0.25   | 0.98  | 0.71 | 0.78 | 63.20     |
|                 | CAPE              | 0.11      | 0.11   | 0.00 | 0.04 | 0.85   | 0.86  | 0.22 | 0.23 | 612.88    |
|                 | PlaneRecNet       | 0.38      | 0.57   | 0.00 | 0.00 | 0.43   | 0.62  | 0.61 | 0.67 | 29.51     |
|                 | SAM               | 0.03      | 0.58   | 0.00 | 0.08 | 0.32   | 0.96  | 0.63 | 0.68 | 0.72      |
|                 | GroundingDINO+SAM | 0.32      | 0.68   | 0.00 | 0.00 | 0.31   | 0.68  | 0.75 | 0.82 | 1.78      |
|                 | ours              | 0.75      | 0.78   | 0.00 | 0.04 | 0.18   | 0.21  | 0.88 | 0.92 | 932.60    |
| TUM cabinet     | DDPFF             | 0.08      | 0.65   | 0.00 | 0.05 | 0.29   | 0.91  | 0.86 | 0.92 | 42.71     |
|                 | CAPE              | 0.76      | 0.76   | 0.00 | 0.02 | 0.23   | 0.21  | 0.87 | 0.93 | 453.44    |
|                 | PlaneRecNet       | 0.89      | 0.86   | 0.01 | 0.01 | 0.11   | 0.09  | 0.88 | 0.94 | 30.11     |
|                 | SAM               | 0.39      | 0.71   | 0.03 | 0.08 | 0.10   | 0.53  | 0.88 | 0.92 | 0.58      |
|                 | GroundingDINO+SAM | 0.71      | 0.27   | 0.21 | 0.00 | 0.47   | 0.07  | 0.81 | 0.89 | 1.81      |
|                 | ours              | 0.90      | 0.87   | 0.00 | 0.01 | 0.12   | 0.09  | 0.89 | 0.94 | 746.53    |
</details>


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
