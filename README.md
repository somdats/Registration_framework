# Registration_framework
The given framework contains source code to process 3D Point Cloud using PCL functionalities along with additional implementation. The following algorithms are implemented as part of the library
1) Read/Write PLY Files and PCT Files( Vialux 3D Scanner format)
2) Correspondence Search ALgorithms for ICP Based Registration
 2.1) KDtree based Closest Point 
 2.2) MLS based Correspondence
 2.3) Correspondence based on Reverse Calibration
3) feature Estimation
3.1) Normation Estimation of POint Clouds
 3.1.1)Using Image based Neighborhood (for range image)
 3.1.2) Direct from Point cloud using PCA based technique
 3.1.3) Point Normals From Triangular Mesh
                       



The dependency of the code are:
PCL,
Boost,
Eigen,
FlANN,
VTK,

DataStructure:Uses PCL's core storage structure

Current Features:
Search Algorithms
Feature Estimation on Point Cloud
Registration ALgrotihm ( ICP Based with various discrete correspondence finding)

Evaluation metric Available
Writes file in .ply format
