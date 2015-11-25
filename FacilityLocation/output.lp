Maximize
  Demand_0 + Demand_1 + Demand_2
Subject To
 Demand_i_is_covered_0: - Demand_0 + Open_0 + Open_1 >= 0
 Demand_i_is_covered_1: - Demand_1 + Open_0 + Open_1 >= 0
 Demand_i_is_covered_2: - Demand_2 + Open_2 >= 0
 R3: Open_0 + Open_1 + Open_2 <= 2
Bounds
Binaries
 Demand_0 Demand_1 Demand_2 Open_0 Open_1 Open_2
End
