# Planning Persuasive Trajectories Based on a Leader-Follower Game Model
This repo host the parameters and the source code for manuscript that is accepted by MECC 2025

## Requirements
MATLAB 2023b and above

## Usage
Once clone the branch, navigate to the root directory of the repo, and add it to the path `addpath('.')`

Before generating the data, please run the unit test to make sure the code is working properly. 
To run the unit test, run [`runAllUnitTests.m`](runAllUnitTests.m)

To generate all the raw data, run [`generate_result_mecc_2025.m`](generate_result_mecc_2025.m)

Then to generate the summary data, run [`stats_mecc_2025.m`](stats_mecc_2025.m), to generate the plots, run [`plotter_mecc_2025.m`](plotter_mecc_2025.m)

Remark: Parallel results may be machine dependent and may not be completely reproducible. For this reason only the serial results are included in the paper.

## Contact: 
[Chaozhe He](https://www.buffalo.edu/~chaozheh/) \
Assistant Professor \
Department of Mechanical and Aerospace Engineering \
University at Buffalo, The State University of New York
