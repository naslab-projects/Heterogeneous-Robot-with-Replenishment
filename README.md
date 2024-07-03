# AUV-USV Mobile Charging

The mission employs AUVs capable of reaching a maximum speed of 3 km/h for 12 hours, incorporating a safety margin of 2 hours. USVs, on the other hand, are bound by a speed limit of 16 km/h, and each rendezvous necessitates an 8-hour battery charging process. Our objective is to minimize the overall mission time, and optimized trajectories of AUVs and USVs through Algorithm 1.

<p align="center">
  <img src="https://github.com/AlexWUrobot/MTSP/blob/main/Algorithm1_v2.PNG" alt="Dijkstra-GA AUV Mobile Charging">
  <br>
  <em> Algorithm 1. Dijkstra-GA AUV Mobile Charging</em>
</p>

# UAV-USV Time-Sequence Charging

The mission utilizes UAVs with two distinct flight modes: Work mode and Travel mode. During the Work mode, the UAV explores the area at a speed of 20 km/h, while in Travel mode, it flies to the start point or charging stations at a speed of 60 km/h. Each flight is limited to a maximum of 0.7 hours to ensure it doesn't exceed the battery capacity. After 0.5 hours of work, the drone heads to the charging stations. Each USV can charge a maximum of two UAVs simultaneously, and the charging process takes 2 hours.

Our goal is to minimize the overall mission time and return distance for charging by the Algorithm 2. The mission involves three UAVs covering the same square area. The previous USV rendezvous points serve as time-sequence charger stations, marked by green and orange crosses. The black crosses indicate interruption points where AUVs need to return for charging.

<p align="center">
  <img src="https://github.com/AlexWUrobot/MTSP/blob/main/Algorithm2_v2.PNG"  width="460" height="auto" alt="GA UAV Time-Sequence Charging">
  <br>
  <em> Algorithm 2. GA UAV Time-Sequence Charging</em>
</p>




# How to Run the Code

Stage1 AUV planning
1. "stage1_auv"  includes the below codes and save "auv_mobile_charger_yyyy-mm-dd_HHMMSS.mat"
   - "stage1_auv_planner"       is to run the GA
   - "stage1_auv_plot_path"     is to plot the AUV-USV path
2.  "stage1_auv_plot_battery " is to plot the AUV-USV batteru schedule and save "asv_travel.mat"

To run the record data: 
1. Read the previous data, press F9 in line 327 of "stage1_auv" 
   - load('auv_mobile_charger_2024-01-17_162755.mat');
2. Plot the AUV-USV path, press F9 in line 332 of "stage1_auv" 
   - [traj_segment,time_given_charger, traj_worker_nth] = stage1_auv_plot_path(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger,G,start_node_th,points_to_remove);
3. Run "stage1_auv_plot_battery " is to plot the AUV-USV batteru schedule 

----------------------------------------------------

Stage2 UAV planning, UAV battery life is 0.4 hour
1. "stage2_uav"               includes the below codes and save "uav_time_sequence_charging_yyyy-mm-dd_HHMMSS.mat"
   - "stage2_uav_planner"       is to run the GA
   - "stage2_uav_plot_path"     is to plot the UAV-USV path 
2. "stage2_uav_plot_battery " is to plot the UAV-USV battery schedule


To run the record data: 
1. Read the previous data, press F9 in line 229~230 of "stage2_uav"
   - load('uav_time_sequence_charging_2024-01-20_051116');
   - load('asv_travel.mat');
2. Plot the UAV-USV path, press F9 in line 236 of "stage2_uav"
   - [traj_segment] = stage2_uav_plot_path(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger, asv_xy);
3. Run "stage2_uav_plot_battery " is to plot the UAV-USV battery schedule


# Optimizing the Number of Working Robots

During a major accident, various rescue teams from surrounding cities may possess different types of robots. 
Both the total energy and rescue time are typically constrained. Optimizing the allocation of working robots can be framed as an Unbounded Knapsack Problem (UKP). 
To address this, a dynamic programming method is introduced and detailed in Algorithm 3.

Users have the ability to input parameters such as the total battery capacity ($\Omega$) on USV equipped with mobile chargers, and the characteristics of the robots. These robot parameters include individual battery capacities ($\omega$), maximum traveling distance, and the range of vision. The range of vision can be adjusted by altering the camera of AUVs or the flight height of UAVs. The total coverage range ($\nu$) is determined by multiplying the maximum traveling distance and the range of vision. A set of hypothetical parameters is illustrated in Table 1.

The dynamic programming method takes these inputs and calculates the maximum coverage area for different numbers of heterogeneous vehicles. In Fig.10, the x-axis represents the varied total battery capacity in the mobile chargers (USV), while the left side of the y-axis in Fig. 11 indicates the numbers of different types of working robots. The right side of the y-axis denotes the estimated total coverage mission range.


Currently, we separately optimize the path planning and the number of workers. In future work, we plan to merge this algorithm to monitor the total remaining battery levels of mobile chargers and change the robots' schedules in real time.

<p align="center">
  <img src="https://github.com/AlexWUrobot/Heterogeneous-Multi-Robot-Planning/blob/main/Algorithm3.PNG" alt="Heterogeneous robots' hypothetical parameters">
  <br>
  <em> Algorithm 3. Robot's Number Optimization</em>
</p>



<p align="center">
  <img src="https://github.com/AlexWUrobot/Heterogeneous-Multi-Robot-Planning/blob/main/robot_spec_v4.png" alt="Heterogeneous robots' hypothetical parameters">
  <br>
  <em> Table 1. Heterogeneous robots' hypothetical parameters</em>
</p>



<p align="center">
  <img src="https://github.com/AlexWUrobot/Heterogeneous-Multi-Robot-Planning/blob/main/UAV_cover_v4.png" alt="">
  <em>Fig 10. Dynamic programming optimizes the number of UAVs under various battery capacities.</em>
</p>



<p align="center">
  <img src="https://github.com/AlexWUrobot/Heterogeneous-Multi-Robot-Planning/blob/main/AUV_cover_v4.png" alt="">
  <em>Fig 11. Dynamic programming optimizes the number of AUVs under various battery capacities</em>
</p>


