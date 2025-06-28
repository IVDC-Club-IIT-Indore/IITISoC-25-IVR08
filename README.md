# IITISoC-25-IVR0xx PS Title


# Coordinated Dual Drone for Coverage Irrigation

We focus on deploying two drones handling scanning of the field to accumulate the data of water requirement and also irrigation in a way that time minimisation and battery optimisation is our ultimate goal


## Geotagger Node

This involves placing of Geotags in a predefined manner using a type of coverage method to scan the whole area using the dual drone efficiently

1. Parsing the given KML file and dividing the area by half and assigning each area to a single drone.
2. Each drone does scanning in the halves in Lawnmover style.
3. While performing Lawnmover, it places Geotags in such a manner that they are 25m apart.
4. Finally we export the Geotags in the form of **json** file, where it has the below format:
```json
    "sector_id": 2,
    "drone_id": 1,
    "center_latitude": 22.5270074,
    "center_longitude": 75.9149487,
    "timestamp": "2025-06-18 07:33:22",
    "severity": 3.59
```
5. This json file is used as input for our Path Planner Node
# ACO-Driven Path Optimization for Precision Agriculture

## Overview

This module implements a bio-inspired Ant Colony Optimization (ACO) algorithm specifically designed for agricultural path planning in precision agriculture drone systems. The system addresses efficient waypoint traversal while considering crop damage minimization, terrain constraints, and operational severity factors.

## Problem Formulation

### Multi-Objective Optimization Challenge

Agricultural path planning differs from traditional Traveling Salesman Problem (TSP) applications by incorporating agricultural-specific constraints:

**Primary Optimization Factors:**
```
P(i,j) = [τ(i,j)]^α · [η(i,j)]^β · [S(j)]^γ / Σ[τ(i,k)]^α · [η(i,k)]^β · [S(k)]^γ
```

Where:
- **τ(i,j)**: Pheromone intensity between waypoints i and j
- **η(i,j) = 1/d(i,j)**: Heuristic information (inverse distance)
- **S(j)**: Normalized severity factor for agricultural priority
- **α, β, γ**: Influence parameters for algorithm tuning

This probability function enables intelligent waypoint selection that balances distance efficiency, learned experience (pheromones), and agricultural urgency.

## Core Algorithms

### Algorithm 1: ACO Path Optimization
```
procedure ACO_OPTIMIZE(waypoints, severity_factors)
    initialize_pheromone_matrix()
    best_path = null
    best_cost = infinity
    
    for iteration = 1 to max_iterations do
        paths = []
        costs = []
        
        for ant = 1 to num_ants do
            current_path = construct_path_probabilistic(waypoints)
            current_cost = calculate_path_cost(current_path)
            paths.append(current_path)
            costs.append(current_cost)
            
            if current_cost < best_cost then
                best_path = current_path
                best_cost = current_cost
            end if
        end for
        
        update_pheromones(paths, costs)
    end for
    
    return optimize_with_2opt(best_path)
end procedure

procedure CONSTRUCT_PATH_PROBABILISTIC(waypoints)
    path = [random_start_waypoint()]
    unvisited = waypoints - path[0]
    
    while unvisited not empty do
        current = path.last()
        probabilities = calculate_probabilities(current, unvisited)
        next_waypoint = select_probabilistic(unvisited, probabilities)
        path.append(next_waypoint)
        unvisited.remove(next_waypoint)
    end while
    
    return path
end procedure
```

**Purpose**: Bio-inspired optimization for agricultural path planning
**Key Features**:
- Multi-objective probability-based selection
- Pheromone trail learning mechanism
- Agricultural severity factor integration

### Algorithm 2: Probability Calculation with Agricultural Weighting
```
procedure CALCULATE_PROBABILITIES(current, unvisited)
    probabilities = []
    
    for waypoint in unvisited do
        pheromone = pheromone_matrix[current][waypoint]
        distance = calculate_distance(current, waypoint)
        heuristic = 1 / (distance + epsilon)
        severity = normalized_severity[waypoint]
        
        desirability = (pheromone^α) × (heuristic^β) × (severity^γ)
        probabilities.append(desirability)
    end for
    
    return normalize(probabilities)
end procedure
```

**Purpose**: Multi-factor waypoint selection considering agricultural priorities
**Key Features**:
- Pheromone influence (learned experience)
- Distance optimization (efficiency)
- Severity weighting (agricultural urgency)

### Algorithm 3: Pheromone Update Mechanism
```
procedure UPDATE_PHEROMONES(paths, costs)
    // Evaporation phase
    for i = 1 to num_waypoints do
        for j = 1 to num_waypoints do
            pheromone_matrix[i][j] *= (1 - evaporation_rate)
        end for
    end for
    
    // Reinforcement phase
    for k = 1 to num_paths do
        pheromone_deposit = Q / costs[k]
        path = paths[k]
        
        for i = 1 to len(path) - 1 do
            from = path[i]
            to = path[i + 1]
            pheromone_matrix[from][to] += pheromone_deposit
            pheromone_matrix[to][from] += pheromone_deposit
        end for
    end for
end procedure
```

**Purpose**: Dynamic learning through pheromone trail management
**Key Features**:
- Evaporation prevents premature convergence
- Quality-based reinforcement
- Bidirectional pheromone deposits

### Algorithm 4: 2-opt Local Search Enhancement
```
procedure TWO_OPT(path, dist_matrix)
    improved = True
    while improved do
        improved = False
        for i = 1 to len(path) - 2 do
            for j = i + 1 to len(path) do
                if j - i ≠ 1 then
                    new_path = reverse_segment(path, i, j)
                    new_cost = calculate_cost(new_path)
                    if new_cost < current_cost then
                        path = new_path
                        improved = True
                    end if
                end if
            end for
        end for
    end while
    return path
end procedure
```

**Purpose**: Local optimization for ACO-generated paths
**Key Features**:
- Edge-swapping optimization
- Iterative improvement until convergence
- Path crossing elimination

## Spatial Partitioning Strategy

### Clustering-Based Field Division

**KMeans Ghost Point Generation:**
Large agricultural fields require intelligent spatial partitioning to enable parallel processing and manageable optimization complexity.

**Process Flow:**
1. **Coordinate Extraction**: Extract latitude/longitude from geotag data
2. **Clustering Application**: Apply KMeans algorithm with configurable cluster count
3. **Ghost Point Generation**: Create representative points for each cluster
4. **Nearest Neighbor Assignment**: Partition geotags based on proximity to ghost points

**Mathematical Foundation:**
- Grid size calculation with Earth curvature correction:
  - Δlat = grid_size / 111000
  - Δlon = grid_size / (111000 · cos(rad(lat_avg)))

## Enhanced ACO Implementation

### Probability Calculation with Agricultural Weighting

**Severity Normalization Process:**
1. Extract raw severity values from agricultural data
2. Apply min-max normalization: `norm_severity = 1 + (raw - min) / (max - min)`
3. Calculate desirability: `(pheromone^α) × (heuristic^β) × (severity^γ)`
4. Normalize probabilities for selection

**Adaptive Factors:**
- **Pheromone Influence (α)**: Controls learning from successful routes
- **Distance Preference (β)**: Balances efficiency vs. exploration  
- **Severity Weighting (γ)**: Prioritizes agricultural urgency

### Pheromone Update Mechanism

**Dual-Phase Update Process:**

**Phase 1 - Evaporation:**
- Reduces all pheromone values: `pheromone *= (1 - ρ)`
- Prevents premature convergence to suboptimal solutions
- Maintains exploration capability

**Phase 2 - Reinforcement:**
- Deposit pheromones on successful paths: `deposit = Q / path_cost`
- Stronger reinforcement for better solutions
- Bidirectional pheromone application

## Performance Analysis

### Computational Complexity

**Individual Component Complexity:**
- **ACO Algorithm**: `T_ACO = O(n² · m · t)`
- **2-opt Enhancement**: `T_2-opt = O(n²)`
- **Total Complexity**: `T_total = T_ACO + T_2-opt`

**Scalability Factors:**
- n: Number of waypoint nodes
- m: Number of artificial ants
- t: Algorithm iterations

### Experimental Performance Metrics

**8-Sector Field Partitioning Results:**

| Partition | Node Count | Optimization Time (s) 
|-----------|------------|----------------------|
| 1         | 102        | 58.46                | 
| 2         | 106        | 59.80                | 
| 3         | 102        | 58.23                | 
| 4         | 111        | 63.87                | 
| 5         | 84         | 45.49                | 
| 6         | 121        | 70.29                | 
| 7         | 88         | 47.72                | 
| 8         | 108        | 61.71                | 




## System Integration

### ROS-Based Architecture

**Communication Interface:**
- Standardized ROS topics for inter-node communication
- Service-based coordination for mission synchronization
- Modular design enabling component independence

### MAVROS Integration

**Flight Controller Interface:**
- Position control through `/mavros/setpoint_position/local`
- State monitoring via `/mavros/state`
- Service integration for arming and mode control

## Future Algorithmic Enhancements

### Planned Algorithm Improvements

1. **Behavior Trees Integration**: 
   - Hierarchical decision-making for complex agricultural scenarios
   - State-based mission adaptation
   - Fault recovery and replanning capabilities

2. **Multi-Drone Sector Division**:
   - Automatic workload distribution algorithms
   - Coordinated path planning between multiple UAVs
   - Load balancing optimization




_**Team Member 1**:  [@UnceasingEagerness](https://github.com/UnceasingEagerness)_

_**Team Member 2**:  [@user2](https://github.com/user2)_

_**Team Member 3**:  [@user3](https://github.com/user3)_

_**Team Member 4**:  [@user4](https://github.com/user4)_

Mentors

_**Nambiar Anand Sreenivasan**:  [@NambiarAnand](https://github.com/NambiarAnand)_

_**R Gopikrishnan**:  [@Rgopikrishnan18](https://github.com/Rgopikrishnan18)_
