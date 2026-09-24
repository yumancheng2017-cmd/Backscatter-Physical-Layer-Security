# Backscatter Physical Layer Security

MATLAB simulation and optimization of a multi-antenna backscatter communication system with physical layer security.

## Overview

This project investigates physical layer security in a monostatic backscatter communication system consisting of a multi-antenna reader, a passive backscatter device, and an eavesdropper.

The objective is to improve the achievable secrecy rate by jointly optimizing the backscatter reflection coefficient and the transmit/receive beamforming design.

## System Model

The considered system includes:

- A multi-antenna reader for signal transmission and reception
- A passive backscatter device
- An eavesdropper
- Forward and backscatter wireless channels
- Backscatter reflection coefficient optimization

The system performance is evaluated in terms of the achievable secrecy rate.

## Optimization Approach

The joint optimization problem is addressed using an alternating optimization framework.

The main techniques include:

- Alternating optimization
- Successive Convex Approximation (SCA)
- Semidefinite Relaxation (SDR)
- Transmit precoding optimization
- Receive combining optimization
- Reflection coefficient optimization

## Tools

- MATLAB
- CVX

## Getting Started

### Requirements

- MATLAB
- CVX

### Running the Simulation

The MATLAB scripts implement the system model and optimization algorithms used to evaluate the secrecy performance of the backscatter communication system.

Further implementation details and simulation results are provided in the repository.

## Results

### Secrecy Rate vs. Eavesdropper Distance

The achievable secrecy rate is evaluated for different numbers of reader antennas as the distance between the tag and the eavesdropper varies.

The optimized design is also compared with a brute-force benchmark.

![Secrecy rate vs. transmission distance](Results/Figure_1.png)

### Optimized Reflection Coefficients

The optimized reflection coefficients are evaluated for different reader antenna configurations over the considered transmission distances.

![Reflection coefficients vs. transmission distance](Results/Figure_2.png)

### Reader Spectral Efficiency

The spectral efficiency of the legitimate reader is evaluated as a function of the tag-to-eavesdropper distance for different numbers of reader antennas.

![Reader spectral efficiency](Results/Figure_3.png)

### Algorithm Convergence

The convergence behavior of the alternating optimization algorithm is illustrated for the case of `N = 4` reader antennas and `d_UE = 20 m`.

The secrecy rate progressively converges over successive iterations, demonstrating the convergence behavior of the proposed SCA/SDR-based optimization procedure.

![Algorithm convergence](Results/Figure_4.png)
