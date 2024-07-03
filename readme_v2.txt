
preplan_GA_drone5 consider moving, wait for drone
preplan_GA_drone4_3 is for uav landing, test_plot_uav_4_3 is to print schedule
preplan_GA_drone4_2 is for asv moving ,  plot_battery_v8 is for print battery




charging_time_start{}


    8.6675    9.2820   -0.2779    9.0554    1.2552   -0.0362



35.7837-35.357 = 0.4267 
just 0.001


35.7475-35.357 = 0.3905
need 0.4268 - 0.3905 = 0.0362

do not need to wait, have time interval rest 0.2417



scheduled_interval =

    8.6675    9.2820   -0.2779    9.0554    1.2552   -0.0362



Index in position 2 exceeds array bounds (must not exceed 20).

Error in ga5_0_drone_CPP (line 308)
                if time_charging > pBatteryLife(s,man_battery_count(s)+1) % if the battery is enough to make this travel, skip this if



--------------------------------------------
AUV-USV
average mission time 100 standard deviation 
travel distance 100 standard deviation 
--------------------------------------------
UAV-USV
average mission time 100 standard deviation 
return distance 100 standard deviation 
travel distance 100 standard deviation 




------------------------------------------

UAV Planning 
preplan_GA_drone7 --> 
    load('uav_time_sequence_charging_2024-01-20_051116');
    load('asv_travel.mat');
    --> ga6_0_drone_CPP
    save uav_time_sequence_charging_yyyy-mm-dd_HHMMSS.mat

    --> preplan_plot_6_drone_CPP (stage2_uav_planner)

Please F9+ 
    % Load data, after load data run 'preplan_plot_6_drone_CPP' and then run 'test_plot_uav_6'
    load('uav_time_sequence_charging_2024-01-20_051116');
    load('asv_travel.mat');
test_plot_uav_6.m

F9
preplan_plot_6_drone_CPP

--------------------------------------------------

preplan_GA_boundary_drone_6 --> 
   save auv_mobile_charger_yyyy-mm-dd_HHMMSS.mat

   --> ga6_0
   --> preplan_plot5_0


plot_battery_v11.m --> save asv_travel.mat


--------------------------------------------------



