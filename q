[33mcommit 2ced37a472026549cc18a52bb79421629ebf1686[m[33m ([m[1;36mHEAD -> [m[1;32mmain[m[33m)[m
Author: Longhao Yang <longhaoy@usc.edu>
Date:   Mon Apr 11 15:28:42 2022 -0700

    Initialized project report Longhao Y

[1mdiff --git a/REPORT.md b/REPORT.md[m
[1mnew file mode 100644[m
[1mindex 0000000..23c4c82[m
[1m--- /dev/null[m
[1m+++ b/REPORT.md[m
[36m@@ -0,0 +1,104 @@[m
[32m+[m[32m# EE 538 Final Project Report[m
[32m+[m
[32m+[m[32mThis project focus on create a map application of location around USC campus by implementing data structure and graph algorithm.[m[41m [m
[32m+[m
[32m+[m[32m## Data Structure[m
[32m+[m
[32m+[m[32mTo create the map, every location in the map is represented by Node. Every node is connected using graph data structure.[m[41m [m
[32m+[m
[32m+[m[32m### Classes[m
[32m+[m
[32m+[m[32m**Node**[m
[32m+[m
[32m+[m[32mAttributes:[m[41m  [m
[32m+[m
[32m+[m[32mid: A unique id assign to each point[m[41m  [m
[32m+[m[32mlat: Latitude of this location[m[41m  [m
[32m+[m[32mlon: Longitude of this location[m[41m  [m
[32m+[m[32mname: Name of this location[m[41m  [m
[32m+[m[32mneighbors: List of the ids of all neighbor points[m[41m  [m
[32m+[m[32mattributes: List of the attributes of the location[m[41m  [m
[32m+[m
[32m+[m[32m```cpp[m
[32m+[m[32mclass Node {[m
[32m+[m[32m  public:[m
[32m+[m[32m    Node(){};[m
[32m+[m[32m    Node(const Node &n){id = n.id; lat = n.lat; lon = n.lon; name = n.name; neighbors = n.neighbors; attributes = n.attributes;};[m
[32m+[m[32m    std::string id;    // A unique id assign to each point[m
[32m+[m[32m    double lat;        // Latitude[m
[32m+[m[32m    double lon;        // Longitude[m
[32m+[m[32m    std::string name;  // Name of the location. E.g. "Bank of America".[m
[32m+[m[32m    std::vector<std::string> neighbors;  // List of the ids of all neighbor points.[m
[32m+[m[32m    std::unordered_set<std::string> attributes;  // List of the attributes of the location.[m
[32m+[m[32m};[m
[32m+[m[32m```[m
[32m+[m
[32m+[m[32m**Trojan Map**[m
[32m+[m
[32m+[m[32mAttributes:[m[41m  [m
[32m+[m
[32m+[m[32mdata: A map stores the id of location as key and its Node as value[m
[32m+[m
[32m+[m[32m*Function below are implementations of the map*[m
[32m+[m
[32m+[m
[32m+[m[32m## Phase 1 report[m[41m  [m
[32m+[m
[32m+[m[32m**GetLat()**[m[41m  [m
[32m+[m[32mTime Complexity: O(1)[m[41m  [m
[32m+[m[32mSpace Complexity: O(1)[m[41m  [m
[32m+[m[32mGet the Latitude of a Node given its id[m[41m  [m
[32m+[m
[32m+[m[32m**GetLon()**[m[41m  [m
[32m+[m[32mTime Complexity: O(1)[m[41m  [m
[32m+[m[32mSpace Complexity: O(1)[m[41m  [m
[32m+[m[32mGet the Longitude of a Node given its id[m
[32m+[m
[32m+[m[32m**GetName()**[m[41m  [m
[32m+[m[32mTime Complexity: O(n)[m[41m  [m
[32m+[m[32mSpace Complexity: O(1)[m[41m  [m
[32m+[m[32mGet the name of a Node given its id[m
[32m+[m
[32m+[m[32m**GetID**[m[41m  [m
[32m+[m[32mTime Complexity: O(1)[m[41m  [m
[32m+[m[32mSpace Complexity: O(1)[m[41m  [m
[32m+[m[32mGet the id given its name[m
[32m+[m
[32m+[m[32m**GetNeighborIDs**[m[41m  [m
[32m+[m[32mTime Complexity: O(1)[m[41m  [m
[32m+[m[32mSpace Complexity: O(1)[m[41m  [m
[32m+[m[32mGet the neighbor ids of a Node[m
[32m+[m
[32m+[m[32m**GetPosition**[m[41m  [m
[32m+[m[32mTime Complexity: O(1)[m[41m  [m
[32m+[m[32mSpace Complexity: O(1)[m[41m  [m
[32m+[m[32mGiven a location name, return its latitude and longitude. If id does not exist, return (-1, -1)[m[41m  [m
[32m+[m
[32m+[m[32m**CalculateEditDistance**[m
[32m+[m[32mTime Complexity: O(n*m)[m[41m  [m
[32m+[m[32mSpace Complexity: O(n*m)[m[41m  [m
[32m+[m[32m*n and m are the length of two words*[m[41m  [m
[32m+[m[32mCalculate edit distance between two location names, that is how many substitution, deletion, and addition to modify one word to another.[m[41m [m
[32m+[m
[32m+[m[32m**FindClosestName**[m
[32m+[m[32mTime Complexity: `O(n*m*d)`[m[41m  [m
[32m+[m[32mSpace Complexity: `O(n*m*d)`[m[41m  [m
[32m+[m[32m*n and m are the length of two words, d is the number of all data nodes*[m[41m  [m
[32m+[m[32mIterate through the map and find the name with smallest edit distance.[m[41m  [m
[32m+[m
[32m+[m[32m**Autocomplete**[m[41m  [m
[32m+[m[32mTime Complexity: O(n)[m[41m  [m
[32m+[m[32mSpace Complexity: O(n)[m[41m  [m
[32m+[m[32mReturns a vector of names given a partial name.[m[41m  [m
[32m+[m[32mRun time testing:[m[41m  [m
[32m+[m[32mInput: `ch`[m[41m  [m
[32m+[m[32mOutput: 11 matches[m[41m  [m
[32m+[m[32mTime taken by function: 8ms[m[41m   [m
[32m+[m
[32m+[m[32m## Phase 2 report[m
[32m+[m
[32m+[m[32m**CalculateDistance**[m
[32m+[m
[32m+[m[32m**CalculatePathLength**[m
[32m+[m
[32m+[m[32m**CalculateShortestPath_Dijkstra**[m
\ No newline at end of file[m
